import os
from datetime import datetime

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import UInt16

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


class VectornavMonitor(Node):
    def __init__(self):
        super().__init__("vectornav_monitor")

        self.start_time = self.get_clock().now()

        self.ins_hex = None
        self.gnss1_hex = None
        self.gnss2_hex = None

        self.startup_phase_str = f"{C_YELLOW}Waiting for INS data...{C_RESET}"
        self.ins_body = STALE_INS_BODY
        self.gnss1_body = STALE_GNSS_BODY
        self.gnss2_body = STALE_GNSS_BODY

        self.ins_last_time = None
        self.gnss1_last_time = None
        self.gnss2_last_time = None

        self.ins_last_timestamp_str = "Never"
        self.gnss1_last_timestamp_str = "Never"
        self.gnss2_last_timestamp_str = "Never"

        self.timeout_duration = Duration(seconds=2)

        self.create_subscription(UInt16, "vectornav/ins_status", self.ins_callback, 10)
        self.create_subscription(UInt16, "vectornav/gnss_status", self.gnss1_callback, 10)
        self.create_subscription(UInt16, "vectornav/gnss2_status", self.gnss2_callback, 10)

        self.create_timer(0.1, self.update_display)

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
        self.ins_last_time = self.get_clock().now()
        self.ins_last_timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.ins_hex = msg.data

        v = msg.data & 0b0000001101110111
        mode = (v >> 0) & 0x03
        gnss_fix = (v >> 2) & 0x01
        imu_err = (v >> 4) & 0x01
        mag_pres_err = (v >> 5) & 0x01
        gnss_err = (v >> 6) & 0x01
        gnss_compass = (v >> 8) & 0x03

        self.startup_phase_str = self.determine_startup_phase(mode, gnss_fix, gnss_compass)

        self.ins_body = (
            f"  {'Mode:':<20}{MODE_ENUM.get(mode, 'UNKNOWN')}\n"
            f"  {'GnssFix:':<20}{TXT_YES if gnss_fix else TXT_NO}\n"
            f"  {'ImuErr:':<20}{TXT_ERR if imu_err else TXT_OK}\n"
            f"  {'MagPresErr:':<20}{TXT_ERR if mag_pres_err else TXT_OK}\n"
            f"  {'GnssErr:':<20}{TXT_ERR if gnss_err else TXT_OK}\n"
            f"  {'GnssCompassFix:':<20}{GNSS_COMPASS_FIX_ENUM.get(gnss_compass, f'UNKNOWN({gnss_compass})')}"
        )

    def gnss1_callback(self, msg: UInt16):
        self.gnss1_last_time = self.get_clock().now()
        self.gnss1_last_timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.gnss1_hex = msg.data
        self.gnss1_body = self.format_gnss_body(msg.data)

    def gnss2_callback(self, msg: UInt16):
        self.gnss2_last_time = self.get_clock().now()
        self.gnss2_last_timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.gnss2_hex = msg.data
        self.gnss2_body = self.format_gnss_body(msg.data)

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

    def build_display_block(self, title, hex_val, last_time_ros, last_time_str, body_str, stale_body_str, now_ros):
        if last_time_ros is None:
            return f"--- {title} {C_YELLOW}[STALE]{C_RESET} ---\n  {'Last received:':<20}Never\n{stale_body_str}"

        delta = now_ros - last_time_ros
        delta_sec = delta.nanoseconds / 1e9
        is_stale = delta_sec > (self.timeout_duration.nanoseconds / 1e9)

        header_title = f"{title} {C_YELLOW}[STALE]{C_RESET}" if is_stale else f"{title} 0x{hex_val:04X}"
        display_body = stale_body_str if is_stale else body_str

        time_ago_str = f"({delta_sec:.1f}s ago)"
        if is_stale:
            time_ago_str = f"{C_YELLOW}{time_ago_str}{C_RESET}"

        return f"--- {header_title} ---\n  {'Last received:':<20}{last_time_str} {time_ago_str}\n{display_body}"

    def update_display(self):
        os.system("")

        now = self.get_clock().now()

        if self.ins_last_time is None:
            phase_display = f"{C_YELLOW}Waiting for INS data...{C_RESET}"
        else:
            delta = now - self.ins_last_time
            if delta.nanoseconds / 1e9 > (self.timeout_duration.nanoseconds / 1e9):
                phase_display = f"{C_YELLOW}[STALE] Connection Lost{C_RESET}"
            else:
                phase_display = self.startup_phase_str

        uptime_delta = now - self.start_time
        uptime_total_seconds = int(uptime_delta.nanoseconds / 1e9)
        hours, remainder = divmod(uptime_total_seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        uptime_str = f"{hours:02d}:{minutes:02d}:{seconds:02d}"

        ins_display = self.build_display_block(
            "InsStatus",
            self.ins_hex,
            self.ins_last_time,
            self.ins_last_timestamp_str,
            self.ins_body,
            STALE_INS_BODY,
            now,
        )
        gnss1_display = self.build_display_block(
            "GNSS1 Status",
            self.gnss1_hex,
            self.gnss1_last_time,
            self.gnss1_last_timestamp_str,
            self.gnss1_body,
            STALE_GNSS_BODY,
            now,
        )
        gnss2_display = self.build_display_block(
            "GNSS2 Status",
            self.gnss2_hex,
            self.gnss2_last_time,
            self.gnss2_last_timestamp_str,
            self.gnss2_body,
            STALE_GNSS_BODY,
            now,
        )

        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        # fmt: off
        print("\033[H\033[2J", end="")
        print("==========================================================================")
        print(f"  Vectornav Monitor | Live Time: {C_CYAN}{current_time}{C_RESET} | Uptime: {C_GREEN}{uptime_str}{C_RESET}")
        print("==========================================================================")
        print(f"  System Phase: {phase_display}")
        print("==========================================================================\n")
        print(ins_display)
        print("\n" + gnss1_display)
        print("\n" + gnss2_display)
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
