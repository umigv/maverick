from __future__ import annotations

import ctypes
import os
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum, IntEnum
from typing import Any, Generic, TypeVar

import utils.lifecycle
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, Float32MultiArray, UInt16

T = TypeVar("T")


def green(s: str) -> str:
    return f"\033[92m{s}\033[0m"


def red(s: str) -> str:
    return f"\033[91m{s}\033[0m"


def yellow(s: str) -> str:
    return f"\033[93m{s}\033[0m"


def cyan(s: str) -> str:
    return f"\033[96m{s}\033[0m"


TEXT_YES = green("Yes")
TEXT_NO = red("No")
TEXT_OK = green("OK")
TEXT_ERR = red("ERROR")

CHECKMARK_OK = green("[✓]")
CHECKMARK_NO = red("[✗]")
CHECKMARK_UNKNOWN = yellow("[?]")

# A topic is considered stale once this many seconds pass without a fresh message.
STALE_TIMEOUT_SEC = 2.0


# INS Status register fields - VectorNav VN300 ICD section 2.7.1
class Mode(IntEnum):
    NOT_TRACKING = 0
    ALIGNING = 1
    TRACKING = 2
    GNSS_LOST = 3

    @classmethod
    def label(cls, mode: int) -> str:
        labels: dict[int, str] = {
            Mode.NOT_TRACKING: red("NotTracking - INS filter non-operational, outputs invalid"),
            Mode.ALIGNING: yellow("Aligning - Position/velocity valid, attitude not valid"),
            Mode.TRACKING: green("Tracking - All outputs valid"),
            Mode.GNSS_LOST: red("GnssLost - Extended GNSS outage, only attitude valid"),
        }
        return labels.get(mode, f"UNKNOWN({mode})")


class GnssCompassFix(IntEnum):
    NO_FIX = 0
    FIX = 2  # value 1 is unused per the ICD
    FIX_AIDING = 3

    @classmethod
    def label(cls, fix: int) -> str:
        labels: dict[int, str] = {
            GnssCompassFix.NO_FIX: red("NoFix - Not reporting a solution"),
            GnssCompassFix.FIX: yellow("Fix - Reporting heading, not aiding INS"),
            GnssCompassFix.FIX_AIDING: green("FixAiding - Reporting heading and aiding INS"),
        }
        return labels.get(fix, f"UNKNOWN({fix})")


# GNSS Status register field - VectorNav VN300 ICD section 2.5.17
class DataSource(IntEnum):
    DISABLED = 0
    INTERNAL_A = 1
    INTERNAL_B = 2
    EXTERNAL = 3

    @classmethod
    def label(cls, source: int) -> str:
        labels: dict[int, str] = {
            DataSource.DISABLED: red("Disabled"),
            DataSource.INTERNAL_A: green("InternalA"),
            DataSource.INTERNAL_B: green("InternalB"),
            DataSource.EXTERNAL: cyan("External"),
        }
        return labels.get(source, f"UNKNOWN({source})")


# INS Status bitfield - VectorNav VN300 ICD section 2.7.1
# LittleEndianStructure + from_buffer_copy(to_bytes("little")) pins LSB-first packing, so decoding is identical on
# little- and big-endian hosts.
# TODO: Change to LittleEndianUnion once 3.14+
class InsStatusReg(ctypes.LittleEndianStructure):
    _fields_ = [  # noqa: RUF012 (ctypes requires _fields_ as a plain list)
        ("mode", ctypes.c_uint16, 2),  # 0-1
        ("gnss_fix", ctypes.c_uint16, 1),  # 2
        ("_reserved3", ctypes.c_uint16, 1),  # 3
        ("imu_err", ctypes.c_uint16, 1),  # 4
        ("mag_pres_err", ctypes.c_uint16, 1),  # 5
        ("gnss_err", ctypes.c_uint16, 1),  # 6
        ("_reserved7", ctypes.c_uint16, 1),  # 7
        ("gnss_compass", ctypes.c_uint16, 2),  # 8-9
        ("_reserved", ctypes.c_uint16, 6),  # 10-15
    ]

    @classmethod
    def from_uint16(cls, value: int) -> InsStatusReg:
        return cls.from_buffer_copy(value.to_bytes(2, "little"))


# GNSS Status bitfield - VectorNav VN300 ICD section 2.5.17.
class GnssStatusReg(ctypes.LittleEndianStructure):
    _fields_ = [  # noqa: RUF012 (ctypes requires _fields_ as a plain list)
        ("enabled", ctypes.c_uint16, 1),  # 0
        ("operational", ctypes.c_uint16, 1),  # 1
        ("fix", ctypes.c_uint16, 1),  # 2
        ("antenna_signal_err", ctypes.c_uint16, 1),  # 3
        ("used_for_nav", ctypes.c_uint16, 1),  # 4
        ("used_for_compass", ctypes.c_uint16, 1),  # 5
        ("_reserved6", ctypes.c_uint16, 2),  # 6-7
        ("data_source", ctypes.c_uint16, 2),  # 8-9
        ("_reserved10", ctypes.c_uint16, 1),  # 10
        ("used_for_nav_curr", ctypes.c_uint16, 1),  # 11
        ("pps_used_for_time", ctypes.c_uint16, 1),  # 12
        ("_reserved13", ctypes.c_uint16, 3),  # 13-15
    ]

    @classmethod
    def from_uint16(cls, value: int) -> GnssStatusReg:
        return cls.from_buffer_copy(value.to_bytes(2, "little"))


# Field order matches the GnssCompassSignalHealthStatus array packed by the driver - VectorNav VN300 ICD section 4.8.1.
@dataclass
class SignalHealth:
    pvt_a: float
    rtk_a: float
    cn0_a: float
    pvt_b: float
    rtk_b: float
    cn0_b: float
    com_pvt: float
    com_rtk: float

    @classmethod
    def from_array(cls, d: list[float]) -> SignalHealth:
        return cls(*d[:8])


@dataclass
class TopicState(Generic[T]):
    last_time: Time | None = None
    last_timestamp_str: str = "Never"
    _value: T | None = None

    @property
    def value(self) -> T:
        # Callers guard with is_stale()/last_time before reading; this enforces that invariant.
        assert self._value is not None, "TopicState.value read before any message arrived"
        return self._value

    def mark_received(self, ros_time: Time, value: T) -> None:
        self.last_time = ros_time
        wall_time = datetime.fromtimestamp(ros_time.nanoseconds / 1e9, tz=timezone.utc)
        self.last_timestamp_str = wall_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self._value = value

    def is_stale(self, now: Time, timeout_sec: float = STALE_TIMEOUT_SEC) -> bool:
        return self.last_time is None or (now - self.last_time).nanoseconds / 1e9 > timeout_sec


def _color_pvt(v: float) -> str:
    # Condition coloring recommended in VectorNav VN300 ICD section 4.8.1
    color = green if v >= 12 else yellow if v >= 8 else red
    return color(f"{v:.0f}")


def _color_cn0(v: float) -> str:
    # Condition coloring recommended in VectorNav VN300 ICD section 4.8.1
    color = green if v >= 47 else yellow if v >= 40 else red
    return color(f"{v:.1f}")


def _color_com(v: float) -> str:
    # Condition coloring recommended in VectorNav VN300 ICD section 4.8.1
    color = green if v >= 12 else yellow if v >= 9 else red
    return color(f"{v:.0f}")


def _signal_health_hints(sh: SignalHealth) -> str:
    # Diagnostic interpretations based on VectorNav VN300 ICD section 4.8.1
    hints = []
    if sh.com_rtk < 6:
        hints.append(f"  {red('GNSS compass requires ≥6 common RTK sats')}")

    a_sky = sh.pvt_a < 8 and sh.rtk_a < 6 and sh.cn0_a >= 50
    b_sky = sh.pvt_b < 8 and sh.rtk_b < 6 and sh.cn0_b >= 50
    if a_sky and b_sky:
        hints.append(f"  {yellow('Sky blockage likely (both antennas)')}")
    elif a_sky:
        hints.append(f"  {yellow('Antenna A: sky blockage likely')}")
    elif b_sky:
        hints.append(f"  {yellow('Antenna B: sky blockage likely')}")

    a_jam = sh.pvt_a >= 8 and sh.rtk_a < 6 and sh.cn0_a < 40
    b_jam = sh.pvt_b >= 8 and sh.rtk_b < 6 and sh.cn0_b < 40
    if a_jam and b_jam:
        hints.append(f"  {red('Jamming / indoors / under canopy - check for nearby USB 3.0 cables')}")
    elif a_jam:
        hints.append(f"  {red('Antenna A: local jamming or cable/antenna issue - check for nearby USB 3.0 cables')}")
    elif b_jam:
        hints.append(f"  {red('Antenna B: local jamming or cable/antenna issue - check for nearby USB 3.0 cables')}")

    min_pvt = min(sh.pvt_a, sh.pvt_b)
    if min_pvt >= 6 and sh.com_pvt < min_pvt * 0.6:
        hints.append(f"  {yellow('Few common satellites - sky blockage between antennas?')}")

    if hints:
        hints.append(f"  {cyan('See VectorNav manual section 4.5.2 for GNSS compass troubleshooting')}")

    return ("\n" + "\n".join(hints)) if hints else ""


# INS startup phase from VectorNav VN300 manual section 4.2.
class Phase(Enum):
    INIT = red("Event A: Sensor Initialization")
    GNSS_FIX = cyan("Event B: GNSS Fix Acquired")
    ALIGNING = yellow("Event C: Aligning (Valid Pos/Vel)")
    COMPASS_TRACKING = yellow("Event G1/G2: GNSS Compass Tracking")
    HEADING_CONVERGENCE = cyan("Event D: Heading Convergence")
    TRACKING = green("Event E: Ready / Tracking")
    GNSS_LOST = red("Event Outage: GNSS Lost (Attitude Only)")
    UNKNOWN = yellow("Unknown Phase Transition")

    @property
    def label(self) -> str:
        return self.value

    @classmethod
    def classify(cls, s: InsStatusReg) -> Phase:
        # Transitions taken from VectorNav VN300 manual section 4.2
        # gnss_fix is a 1-bit flag (1 = fix, 0 = no fix)
        match (s.mode, s.gnss_compass, s.gnss_fix):
            case (Mode.GNSS_LOST, _, _):
                return cls.GNSS_LOST
            case (Mode.TRACKING, _, _):
                return cls.TRACKING
            case (Mode.ALIGNING, GnssCompassFix.FIX_AIDING, _):
                return cls.HEADING_CONVERGENCE
            case (Mode.ALIGNING, GnssCompassFix.FIX, _):
                return cls.COMPASS_TRACKING
            case (Mode.ALIGNING, GnssCompassFix.NO_FIX, _):
                return cls.ALIGNING
            case (Mode.NOT_TRACKING, _, 1):
                return cls.GNSS_FIX
            case (Mode.NOT_TRACKING, _, 0):
                return cls.INIT
            case _:
                return cls.UNKNOWN


class VectornavMonitor(Node):
    def __init__(self) -> None:
        super().__init__("vectornav_monitor")

        os.system("")  # enable ANSI escape processing on Windows terminals

        self.start_time = self.get_clock().now()

        self.ins: TopicState[InsStatusReg] = TopicState()
        self.gnss1: TopicState[GnssStatusReg] = TopicState()
        self.gnss2: TopicState[GnssStatusReg] = TopicState()
        self.signal_health: TopicState[SignalHealth] = TopicState()
        self.startup: TopicState[float] = TopicState()
        self.yaw: TopicState[float] = TopicState()

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

    def ins_callback(self, msg: UInt16) -> None:
        self.ins.mark_received(self.get_clock().now(), InsStatusReg.from_uint16(msg.data))

    def gnss1_callback(self, msg: UInt16) -> None:
        self.gnss1.mark_received(self.get_clock().now(), GnssStatusReg.from_uint16(msg.data))

    def gnss2_callback(self, msg: UInt16) -> None:
        self.gnss2.mark_received(self.get_clock().now(), GnssStatusReg.from_uint16(msg.data))

    def signal_health_callback(self, msg: Float32MultiArray) -> None:
        self.signal_health.mark_received(self.get_clock().now(), SignalHealth.from_array(msg.data))

    def startup_status_callback(self, msg: Float32MultiArray) -> None:
        self.startup.mark_received(self.get_clock().now(), msg.data[0])

    def yaw_uncertainty_callback(self, msg: Float32) -> None:
        self.yaw.mark_received(self.get_clock().now(), msg.data)

    def build_transition_checklist(self, now: Time) -> str:
        # Checklist taken from VectorNav VN300 manual section 4.5.2 and empirical observations during testing
        if self.ins.is_stale(now):
            return f"  {yellow('Waiting for INS data...')}"

        match Phase.classify(self.ins.value):
            case Phase.GNSS_LOST:
                return f"  {red('GNSS Lost - waiting for signal re-acquisition')}"

            case Phase.TRACKING:
                return f"  {green('System fully operational')}"

            case Phase.HEADING_CONVERGENCE:
                header = f"  Next → {Phase.TRACKING.label}"

                if self.yaw.is_stale(now):
                    return f"{header}\n    {CHECKMARK_UNKNOWN} Yaw uncertainty < 2°  (data stale)"

                chk = CHECKMARK_OK if self.yaw.value < 2.0 else CHECKMARK_NO
                return f"{header}\n    {chk} Yaw uncertainty < 2°  (current: {self.yaw.value:.2f}°)"

            case Phase.COMPASS_TRACKING:
                header = f"  Next → {Phase.HEADING_CONVERGENCE.label}"

                if self.startup.is_stale(now):
                    return f"{header}\n    {CHECKMARK_UNKNOWN} Startup % = 100%  (data stale)"

                chk = CHECKMARK_OK if self.startup.value >= 100 else CHECKMARK_NO
                return f"{header}\n    {chk} Startup % = 100%  (current: {self.startup.value:.0f}%)"

            case Phase.ALIGNING:
                header = f"  Next → {Phase.COMPASS_TRACKING.label}"
                if self.signal_health.is_stale(now):
                    return f"{header}\n    {CHECKMARK_UNKNOWN} Common RTK sats ≥ 6  (data stale)"

                rtk_ok = self.signal_health.value.com_rtk >= 6
                chk = CHECKMARK_OK if rtk_ok else CHECKMARK_NO
                lines = [header, f"    {chk} Common RTK sats ≥ 6  (current: {self.signal_health.value.com_rtk:.0f})"]
                if rtk_ok:
                    lines.append(
                        f"    {yellow('Still in C despite RTK ≥ 6? Try unplugging and replugging the sensor')}"
                    )
                return "\n".join(lines)

            case Phase.GNSS_FIX:
                return f"  Next → {Phase.ALIGNING.label}\n    Automatic (~1s after GNSS fix)"

            case Phase.INIT:
                return (
                    f"  Next → {Phase.GNSS_FIX.label}\n    {CHECKMARK_NO} GNSS fix  (waiting, ~30-45s with clear sky)"
                )

            case _:
                return f"  {yellow('Unknown phase')}"

    def block_frame(self, title: str, state: TopicState[Any], now: Time, *, show_hex: bool = False) -> tuple[str, bool]:
        if state.last_time is None:
            return f"--- {title} {yellow('[STALE]')} ---\n  {'Last received:':<20}Never", True

        delta_sec = (now - state.last_time).nanoseconds / 1e9
        ago = f"({delta_sec:.1f}s ago)"
        if state.is_stale(now):
            header = f"--- {title} {yellow('[STALE]')} ---"
            ago = yellow(ago)
        elif show_hex:
            raw = int.from_bytes(bytes(state.value), "little")
            header = f"--- {title} 0x{raw:04X} ---"
        else:
            header = f"--- {title} ---"

        return f"{header}\n  {'Last received:':<20}{state.last_timestamp_str} {ago}", state.is_stale(now)

    def build_ins(self, now: Time) -> str:
        frame, stale = self.block_frame("InsStatus", self.ins, now, show_hex=True)
        if stale:
            return f"{frame}\n" + "\n".join([""] * 6)

        s = self.ins.value
        body = "\n".join(
            [
                f"  {'Mode:':<20}{Mode.label(s.mode)}",
                f"  {'GnssFix:':<20}{TEXT_YES if s.gnss_fix else TEXT_NO}",
                f"  {'ImuErr:':<20}{TEXT_ERR if s.imu_err else TEXT_OK}",
                f"  {'MagPresErr:':<20}{TEXT_ERR if s.mag_pres_err else TEXT_OK}",
                f"  {'GnssErr:':<20}{TEXT_ERR if s.gnss_err else TEXT_OK}",
                f"  {'GnssCompassFix:':<20}{GnssCompassFix.label(s.gnss_compass)}",
            ]
        )

        return f"{frame}\n{body}"

    def build_gnss(self, title: str, state: TopicState[GnssStatusReg], now: Time) -> str:
        frame, stale = self.block_frame(title, state, now, show_hex=True)
        if stale:
            return f"{frame}\n" + "\n".join([""] * 9)

        s = state.value
        body = "\n".join(
            [
                f"  {'Enabled:':<20}{TEXT_YES if s.enabled else TEXT_NO}",
                f"  {'Operational:':<20}{TEXT_YES if s.operational else TEXT_NO}",
                f"  {'Fix:':<20}{TEXT_YES if s.fix else TEXT_NO}",
                f"  {'AntennaSignalErr:':<20}{TEXT_ERR if s.antenna_signal_err else TEXT_OK}",
                f"  {'UsedForNav:':<20}{TEXT_YES if s.used_for_nav else TEXT_NO}",
                f"  {'UsedForCompass:':<20}{TEXT_YES if s.used_for_compass else TEXT_NO}",
                f"  {'DataSource:':<20}{DataSource.label(s.data_source)}",
                f"  {'UsedForNavCurr:':<20}{TEXT_YES if s.used_for_nav_curr else TEXT_NO}",
                f"  {'ppsUsedForTime:':<20}{TEXT_YES if s.pps_used_for_time else TEXT_NO}",
            ]
        )

        return f"{frame}\n{body}"

    def build_signal_health(self, now: Time) -> str:
        frame, stale = self.block_frame("GNSS Compass Signal Health", self.signal_health, now)
        if stale:
            return f"{frame}\n" + "\n".join([""] * 5)

        sh = self.signal_health.value
        body = "\n".join(
            [
                f"  {'PVT Sats:':<20}A={_color_pvt(sh.pvt_a)}  B={_color_pvt(sh.pvt_b)}",
                f"  {'RTK Sats:':<20}A={_color_pvt(sh.rtk_a)}  B={_color_pvt(sh.rtk_b)}",
                f"  {'Highest CN0:':<20}A={_color_cn0(sh.cn0_a)}  B={_color_cn0(sh.cn0_b)}",
                f"  {'Common PVT:':<20}{_color_com(sh.com_pvt)}",
                f"  {'Common RTK:':<20}{_color_com(sh.com_rtk)}",
            ]
        )

        return f"{frame}\n{body}{_signal_health_hints(sh)}"

    def update_display(self) -> None:
        now = self.get_clock().now()

        if self.ins.last_time is None:
            phase_display = yellow("Waiting for INS data...")
        elif self.ins.is_stale(now):
            phase_display = yellow("[STALE] Connection Lost")
        else:
            phase_display = Phase.classify(self.ins.value).label

        uptime_total_seconds = int((now - self.start_time).nanoseconds / 1e9)
        hours, remainder = divmod(uptime_total_seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        uptime_str = f"{hours:02d}:{minutes:02d}:{seconds:02d}"

        # fmt: off
        print("\033[H\033[2J", end="")
        print("==========================================================================")
        print(f"  Vectornav Monitor | Live Time: {cyan(datetime.now(timezone.utc).astimezone().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])} | Uptime: {green(uptime_str)}")
        print("==========================================================================")
        print(f"  System Phase: {phase_display}")
        print(self.build_transition_checklist(now))
        print("==========================================================================\n")
        print(self.build_ins(now))
        print("\n" + self.build_gnss("GNSS1 Status", self.gnss1, now))
        print("\n" + self.build_gnss("GNSS2 Status", self.gnss2, now))
        print("\n" + self.build_signal_health(now))
        # fmt: on


def main() -> None:
    utils.lifecycle.run_node(VectornavMonitor)


if __name__ == "__main__":
    main()
