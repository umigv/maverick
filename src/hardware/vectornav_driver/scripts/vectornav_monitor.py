import os
from dataclasses import dataclass
from datetime import datetime

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, UInt16

C_GREEN = "\033[92m"
C_RED = "\033[91m"
C_YELLOW = "\033[93m"
C_CYAN = "\033[96m"
C_RESET = "\033[0m"

TXT_YES = f"{C_GREEN}Yes{C_RESET}"
TXT_NO = f"{C_RED}No{C_RESET}"
TXT_OK = f"{C_GREEN}OK{C_RESET}"
TXT_ERR = f"{C_RED}ERROR{C_RESET}"

MODE_ENUM = {
    0: f"{C_RED}NotTracking - INS filter non-operational, outputs invalid{C_RESET}",
    1: f"{C_YELLOW}Aligning - Position/velocity valid, attitude not valid{C_RESET}",
    2: f"{C_GREEN}Tracking - All outputs valid{C_RESET}",
    3: f"{C_RED}GnssLost - Extended GNSS outage, only attitude valid{C_RESET}",
}

GNSS_COMPASS_FIX_ENUM = {
    0: f"{C_RED}NoFix - Not reporting a solution{C_RESET}",
    2: f"{C_YELLOW}Fix - Reporting heading, not aiding INS{C_RESET}",
    3: f"{C_GREEN}FixAiding - Reporting heading and aiding INS{C_RESET}",
}

DATA_SOURCE_ENUM = {
    0: f"{C_RED}Disabled{C_RESET}",
    1: f"{C_GREEN}InternalA{C_RESET}",
    2: f"{C_GREEN}InternalB{C_RESET}",
    3: f"{C_CYAN}External{C_RESET}",
}

STALE_INS_BODY = (
    f"  {'Mode:':<20}---\n"
    f"  {'GnssFix:':<20}---\n"
    f"  {'ImuErr:':<20}---\n"
    f"  {'MagPresErr:':<20}---\n"
    f"  {'GnssErr:':<20}---\n"
    f"  {'GnssCompassFix:':<20}---"
)

STALE_SIGNAL_HEALTH_BODY = (
    f"  {'PVT Sats:':<20}A=---  B=---\n"
    f"  {'RTK Sats:':<20}A=---  B=---\n"
    f"  {'Highest CN0:':<20}A=---  B=---\n"
    f"  {'Common PVT:':<20}---\n"
    f"  {'Common RTK:':<20}---"
)

STALE_GNSS_BODY = (
    f"  {'Enabled:':<20}---\n"
    f"  {'Operational:':<20}---\n"
    f"  {'Fix:':<20}---\n"
    f"  {'AntennaSignalErr:':<20}---\n"
    f"  {'UsedForNav:':<20}---\n"
    f"  {'UsedForCompass:':<20}---\n"
    f"  {'DataSource:':<20}---\n"
    f"  {'UsedForNavCurr:':<20}---\n"
    f"  {'ppsUsedForTime:':<20}---"
)

CHK_OK = f"{C_GREEN}[✓]{C_RESET}"
CHK_NO = f"{C_RED}[✗]{C_RESET}"
CHK_UNK = f"{C_YELLOW}[?]{C_RESET}"


def _color_pvt(v: float) -> str:
    s = f"{v:.0f}"
    if v >= 12:
        return f"{C_GREEN}{s}{C_RESET}"
    if v >= 8:
        return f"{C_YELLOW}{s}{C_RESET}"
    return f"{C_RED}{s}{C_RESET}"


def _color_cn0(v: float) -> str:
    s = f"{v:.1f}"
    if v >= 47:
        return f"{C_GREEN}{s}{C_RESET}"
    if v >= 40:
        return f"{C_YELLOW}{s}{C_RESET}"
    return f"{C_RED}{s}{C_RESET}"


def _color_com(v: float) -> str:
    s = f"{v:.0f}"
    if v >= 12:
        return f"{C_GREEN}{s}{C_RESET}"
    if v >= 9:
        return f"{C_YELLOW}{s}{C_RESET}"
    return f"{C_RED}{s}{C_RESET}"


@dataclass
class TopicState:
    stale_body: str
    last_time: object | None = None
    last_timestamp_str: str = "Never"
    hex_val: int | None = None

    def __post_init__(self):
        self.body = self.stale_body

    def mark_received(self, ros_time, wall_time: datetime) -> None:
        self.last_time = ros_time
        self.last_timestamp_str = wall_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


class VectornavMonitor(Node):
    def __init__(self):
        super().__init__("vectornav_monitor")

        self.start_time = self.get_clock().now()

        self._mode: int | None = None
        self._gnss_fix: int | None = None
        self._gnss_compass: int | None = None
        self._com_rtk: float = 0.0
        self._startup_pct: float = 0.0
        self._yaw_uncertainty_deg: float = float("inf")

        self.startup_phase_str = f"{C_YELLOW}Waiting for INS data...{C_RESET}"

        self.ins = TopicState(STALE_INS_BODY)
        self.gnss1 = TopicState(STALE_GNSS_BODY)
        self.gnss2 = TopicState(STALE_GNSS_BODY)
        self.signal_health = TopicState(STALE_SIGNAL_HEALTH_BODY)
        self.startup_last_time = None
        self.yaw_last_time = None

        self.timeout_duration = Duration(seconds=2)

        self.create_subscription(UInt16, "vectornav/ins_status", self.ins_callback, 10)
        self.create_subscription(UInt16, "vectornav/gnss_status", self.gnss1_callback, 10)
        self.create_subscription(UInt16, "vectornav/gnss2_status", self.gnss2_callback, 10)
        self.create_subscription(
            Float32MultiArray, "vectornav/gnss_compass_signal_health", self.signal_health_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, "vectornav/gnss_compass_startup_status", self.startup_status_callback, 10
        )
        self.create_subscription(Float32, "vectornav/yaw_uncertainty", self.yaw_uncertainty_callback, 10)

        self.create_timer(0.1, self.update_display)

    @property
    def _timeout_sec(self) -> float:
        return self.timeout_duration.nanoseconds / 1e9

    def determine_startup_phase(self, mode: int, gnss_fix: int, gnss_compass: int) -> str:
        if mode == 3:
            return f"{C_RED}Event Outage: GNSS Lost (Attitude Only){C_RESET}"
        if mode == 2:
            return f"{C_GREEN}Event E: Ready / Tracking{C_RESET}"
        if mode == 1 and gnss_compass == 3:
            return f"{C_CYAN}Event D: Heading Convergence{C_RESET}"
        if mode == 1 and gnss_compass == 2:
            return f"{C_YELLOW}Event G1/G2: GNSS Compass Tracking{C_RESET}"
        if mode == 1 and gnss_compass == 0:
            return f"{C_YELLOW}Event C: Aligning (Valid Pos/Vel){C_RESET}"
        if mode == 0 and gnss_fix == 1:
            return f"{C_CYAN}Event B: GNSS Fix Acquired{C_RESET}"
        if mode == 0 and gnss_fix == 0:
            return f"{C_RED}Event A: Sensor Initialization{C_RESET}"
        return f"{C_YELLOW}Unknown Phase Transition{C_RESET}"

    def ins_callback(self, msg: UInt16):
        self.ins.mark_received(self.get_clock().now(), datetime.now())
        self.ins.hex_val = msg.data

        v = msg.data & 0b0000001101110111
        mode = (v >> 0) & 0x03
        gnss_fix = (v >> 2) & 0x01
        imu_err = (v >> 4) & 0x01
        mag_pres_err = (v >> 5) & 0x01
        gnss_err = (v >> 6) & 0x01
        gnss_compass = (v >> 8) & 0x03

        self._mode = mode
        self._gnss_fix = gnss_fix
        self._gnss_compass = gnss_compass
        self.startup_phase_str = self.determine_startup_phase(mode, gnss_fix, gnss_compass)

        self.ins.body = (
            f"  {'Mode:':<20}{MODE_ENUM.get(mode, 'UNKNOWN')}\n"
            f"  {'GnssFix:':<20}{TXT_YES if gnss_fix else TXT_NO}\n"
            f"  {'ImuErr:':<20}{TXT_ERR if imu_err else TXT_OK}\n"
            f"  {'MagPresErr:':<20}{TXT_ERR if mag_pres_err else TXT_OK}\n"
            f"  {'GnssErr:':<20}{TXT_ERR if gnss_err else TXT_OK}\n"
            f"  {'GnssCompassFix:':<20}{GNSS_COMPASS_FIX_ENUM.get(gnss_compass, f'UNKNOWN({gnss_compass})')}"
        )

    def gnss1_callback(self, msg: UInt16):
        self._update_gnss(self.gnss1, msg)

    def gnss2_callback(self, msg: UInt16):
        self._update_gnss(self.gnss2, msg)

    def _update_gnss(self, state: TopicState, msg: UInt16) -> None:
        state.mark_received(self.get_clock().now(), datetime.now())
        state.hex_val = msg.data
        state.body = self.format_gnss_body(msg.data)

    def format_gnss_body(self, v: int) -> str:
        enabled = (v >> 0) & 0x01
        operational = (v >> 1) & 0x01
        fix = (v >> 2) & 0x01
        antenna_signal_err = (v >> 3) & 0x01
        used_for_nav = (v >> 4) & 0x01
        used_for_compass = (v >> 5) & 0x01
        data_source = (v >> 8) & 0x03
        used_for_nav_curr = (v >> 11) & 0x01
        pps_used_for_time = (v >> 12) & 0x01

        return (
            f"  {'Enabled:':<20}{TXT_YES if enabled else TXT_NO}\n"
            f"  {'Operational:':<20}{TXT_YES if operational else TXT_NO}\n"
            f"  {'Fix:':<20}{TXT_YES if fix else TXT_NO}\n"
            f"  {'AntennaSignalErr:':<20}{TXT_ERR if antenna_signal_err else TXT_OK}\n"
            f"  {'UsedForNav:':<20}{TXT_YES if used_for_nav else TXT_NO}\n"
            f"  {'UsedForCompass:':<20}{TXT_YES if used_for_compass else TXT_NO}\n"
            f"  {'DataSource:':<20}{DATA_SOURCE_ENUM.get(data_source, f'UNKNOWN({data_source})')}\n"
            f"  {'UsedForNavCurr:':<20}{TXT_YES if used_for_nav_curr else TXT_NO}\n"
            f"  {'ppsUsedForTime:':<20}{TXT_YES if pps_used_for_time else TXT_NO}"
        )

    def signal_health_callback(self, msg: Float32MultiArray):
        self.signal_health.mark_received(self.get_clock().now(), datetime.now())
        d = msg.data
        pvt_a, rtk_a, cn0_a = d[0], d[1], d[2]
        pvt_b, rtk_b, cn0_b = d[3], d[4], d[5]
        com_pvt, com_rtk = d[6], d[7]

        hints = []
        if com_rtk < 6:
            hints.append(f"  {C_RED}GNSS compass requires ≥6 common RTK sats{C_RESET}")

        a_sky = pvt_a < 8 and rtk_a < 6 and cn0_a >= 50
        b_sky = pvt_b < 8 and rtk_b < 6 and cn0_b >= 50
        if a_sky and b_sky:
            hints.append(f"  {C_YELLOW}Sky blockage likely (both antennas){C_RESET}")
        elif a_sky:
            hints.append(f"  {C_YELLOW}Antenna A: sky blockage likely{C_RESET}")
        elif b_sky:
            hints.append(f"  {C_YELLOW}Antenna B: sky blockage likely{C_RESET}")

        a_jam = pvt_a >= 8 and rtk_a < 6 and cn0_a < 40
        b_jam = pvt_b >= 8 and rtk_b < 6 and cn0_b < 40
        if a_jam and b_jam:
            hints.append(f"  {C_RED}Jamming / indoors / under canopy - check for nearby USB 3.0 cables{C_RESET}")
        elif a_jam:
            hints.append(
                f"  {C_RED}Antenna A: local jamming or cable/antenna issue - check for nearby USB 3.0 cables{C_RESET}"
            )
        elif b_jam:
            hints.append(
                f"  {C_RED}Antenna B: local jamming or cable/antenna issue - check for nearby USB 3.0 cables{C_RESET}"
            )

        min_pvt = min(pvt_a, pvt_b)
        if min_pvt >= 6 and com_pvt < min_pvt * 0.6:
            hints.append(f"  {C_YELLOW}Few common satellites - sky blockage between antennas?{C_RESET}")

        if hints:
            hints.append(f"  {C_CYAN}See VectorNav manual §4.5.2 for GNSS compass troubleshooting{C_RESET}")

        self._com_rtk = com_rtk
        hint_str = ("\n" + "\n".join(hints)) if hints else ""

        self.signal_health.body = (
            f"  {'PVT Sats:':<20}A={_color_pvt(pvt_a)}  B={_color_pvt(pvt_b)}\n"
            f"  {'RTK Sats:':<20}A={_color_pvt(rtk_a)}  B={_color_pvt(rtk_b)}\n"
            f"  {'Highest CN0:':<20}A={_color_cn0(cn0_a)}  B={_color_cn0(cn0_b)}\n"
            f"  {'Common PVT:':<20}{_color_com(com_pvt)}\n"
            f"  {'Common RTK:':<20}{_color_com(com_rtk)}{hint_str}"
        )

    def startup_status_callback(self, msg: Float32MultiArray):
        self.startup_last_time = self.get_clock().now()
        self._startup_pct = msg.data[0]

    def yaw_uncertainty_callback(self, msg: Float32):
        self.yaw_last_time = self.get_clock().now()
        self._yaw_uncertainty_deg = msg.data

    def build_transition_checklist(self, now) -> str:
        def stale(t):
            return t is None or (now - t).nanoseconds / 1e9 > self._timeout_sec

        ins_stale = stale(self.ins.last_time)
        sig_stale = stale(self.signal_health.last_time)
        start_stale = stale(self.startup_last_time)
        yaw_stale = stale(self.yaw_last_time)

        if ins_stale:
            return f"  {C_YELLOW}Waiting for INS data...{C_RESET}"

        mode = self._mode
        gnss_compass = self._gnss_compass

        if mode == 3:
            return f"  {C_RED}GNSS Lost - waiting for signal re-acquisition{C_RESET}"

        if mode == 2:
            return f"  {C_GREEN}System fully operational{C_RESET}"

        if mode == 1 and gnss_compass == 3:
            lines = [f"  Next → {C_GREEN}Event E (Tracking){C_RESET}:"]
            if yaw_stale:
                lines.append(f"    {CHK_UNK} Yaw uncertainty < 2°  (data stale)")
            else:
                chk = CHK_OK if self._yaw_uncertainty_deg < 2.0 else CHK_NO
                lines.append(f"    {chk} Yaw uncertainty < 2°  (current: {self._yaw_uncertainty_deg:.2f}°)")
            return "\n".join(lines)

        if mode == 1 and gnss_compass == 2:
            lines = [f"  Next → {C_CYAN}Event D (Heading Convergence){C_RESET}:"]
            if start_stale:
                lines.append(f"    {CHK_UNK} Startup % = 100%  (data stale)")
            else:
                chk = CHK_OK if self._startup_pct >= 100 else CHK_NO
                lines.append(f"    {chk} Startup % = 100%  (current: {self._startup_pct:.0f}%)")
            return "\n".join(lines)

        if mode == 1 and gnss_compass == 0:
            lines = [f"  Next → {C_YELLOW}Event G1/G2 (GNSS Compass Tracking){C_RESET}:"]
            if sig_stale:
                lines.append(f"    {CHK_UNK} Common RTK sats ≥ 6  (data stale)")
            else:
                rtk_ok = self._com_rtk >= 6
                chk = CHK_OK if rtk_ok else CHK_NO
                lines.append(f"    {chk} Common RTK sats ≥ 6  (current: {self._com_rtk:.0f})")
                if rtk_ok:
                    lines.append(
                        f"    {C_YELLOW}Still in C despite RTK ≥ 6? Try unplugging and replugging the sensor{C_RESET}"
                    )
            return "\n".join(lines)

        if mode == 0:
            if self._gnss_fix:
                return f"  Next → {C_CYAN}Event C (Aligning){C_RESET}:\n    Automatic (~1s after GNSS fix)"
            else:
                return (
                    f"  Next → {C_CYAN}Event B (GNSS Fix){C_RESET}:\n"
                    f"    {CHK_NO} GNSS fix  (waiting, ~30-45s with clear sky)"
                )

        return f"  {C_YELLOW}Unknown phase{C_RESET}"

    def build_display_block(self, title: str, state: TopicState, now_ros) -> str:
        if state.last_time is None:
            return f"--- {title} {C_YELLOW}[STALE]{C_RESET} ---\n  {'Last received:':<20}Never\n{state.stale_body}"

        delta_sec = (now_ros - state.last_time).nanoseconds / 1e9
        is_stale = delta_sec > self._timeout_sec

        header_title = (
            f"{title} {C_YELLOW}[STALE]{C_RESET}"
            if is_stale
            else (f"{title} 0x{state.hex_val:04X}" if state.hex_val is not None else title)
        )
        display_body = state.stale_body if is_stale else state.body
        time_ago_str = f"({delta_sec:.1f}s ago)"
        if is_stale:
            time_ago_str = f"{C_YELLOW}{time_ago_str}{C_RESET}"

        return (
            f"--- {header_title} ---\n  {'Last received:':<20}{state.last_timestamp_str} {time_ago_str}\n{display_body}"
        )

    def update_display(self):
        os.system("")

        now = self.get_clock().now()

        if self.ins.last_time is None:
            phase_display = f"{C_YELLOW}Waiting for INS data...{C_RESET}"
        else:
            delta = now - self.ins.last_time
            if delta.nanoseconds / 1e9 > self._timeout_sec:
                phase_display = f"{C_YELLOW}[STALE] Connection Lost{C_RESET}"
            else:
                phase_display = self.startup_phase_str

        uptime_delta = now - self.start_time
        uptime_total_seconds = int(uptime_delta.nanoseconds / 1e9)
        hours, remainder = divmod(uptime_total_seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        uptime_str = f"{hours:02d}:{minutes:02d}:{seconds:02d}"

        # fmt: off
        print("\033[H\033[2J", end="")
        print("==========================================================================")
        print(f"  Vectornav Monitor | Live Time: {C_CYAN}{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}{C_RESET} | Uptime: {C_GREEN}{uptime_str}{C_RESET}")
        print("==========================================================================")
        print(f"  System Phase: {phase_display}")
        print(self.build_transition_checklist(now))
        print("==========================================================================\n")
        print(self.build_display_block("InsStatus", self.ins, now))
        print("\n" + self.build_display_block("GNSS1 Status", self.gnss1, now))
        print("\n" + self.build_display_block("GNSS2 Status", self.gnss2, now))
        print("\n" + self.build_display_block("GNSS Compass Signal Health", self.signal_health, now))
        # fmt: on


def main() -> None:
    rclpy.init()
    node = VectornavMonitor()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
