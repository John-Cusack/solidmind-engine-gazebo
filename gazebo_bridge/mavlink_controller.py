"""MAVLink client for PX4 SITL flight sessions.

The Gazebo bridge holds one ``MavlinkController`` per active flight
session.  The controller owns a single pymavlink connection to PX4 and
the daemon threads needed to keep that connection alive — a receive
loop that decodes telemetry messages and a setpoint streamer that
publishes velocity setpoints at the rate PX4 requires for OFFBOARD
mode (~10 Hz minimum, or PX4 falls back to FAILSAFE).

The controller exposes synchronous high-level commands (``arm``,
``set_offboard_mode``, ``takeoff``, ``set_velocity``, ``land``) so the
bridge runtime — which is itself synchronous — can drive flights
without an asyncio loop.

The threading model:

- The constructor and ``connect()`` run on the caller thread.
- ``connect()`` blocks until the first HEARTBEAT arrives, then starts
  a receive loop and a heartbeat publisher (and, when a setpoint
  stream is active, a setpoint publisher).
- The heartbeat publisher is not optional: it is what identifies us to
  PX4 as a ground station, and PX4 refuses to arm without one.
- Every command method sends a ``COMMAND_LONG`` and synchronously
  waits for the matching ``COMMAND_ACK``.  The receive loop both
  feeds telemetry state and surfaces the ack via an event.

When pymavlink is missing (the ``[drone]`` optional dep was not
installed) ``import gazebo_bridge.mavlink_controller`` still succeeds; the
ImportError is deferred to ``connect()`` so unit tests that never
connect can still import the module.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

logger = logging.getLogger("solidmind.mavlink_controller")


class MavlinkError(Exception):
    """Raised when a MAVLink operation fails."""

    def __init__(self, message: str, *, code: str | None = None) -> None:
        super().__init__(message)
        self.code = code


@dataclass(slots=True)
class FlightTelemetry:
    """Snapshot of the most recent state read from PX4."""

    last_heartbeat_ts: float = 0.0
    custom_mode: int = 0
    base_mode: int = 0
    armed: bool = False
    local_position: tuple[float, float, float] | None = None  # (n, e, d) m
    local_velocity: tuple[float, float, float] | None = None  # m/s


@dataclass(slots=True)
class _Setpoint:
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    vz_mps: float = 0.0
    yaw_rate_rps: float = 0.0


def _import_mavutil() -> tuple[Any, Any]:
    """Lazy import of pymavlink.

    Lifted into a function so unit tests can monkey-patch the import,
    and so importing this module never fails when pymavlink is absent.
    Raises ``MavlinkError`` (not ``ImportError``) so callers see a
    domain-typed failure with a helpful hint.
    """
    try:
        from pymavlink import mavutil
        from pymavlink.dialects.v20 import common as mavlink2  # noqa: F401
    except ImportError as exc:  # noqa: BLE001
        raise MavlinkError(
            "pymavlink is not installed. Run: pip install -e '.[drone]' "
            "from the solidmind-cad repo root.",
            code="PYMAVLINK_MISSING",
        ) from exc
    return mavutil, mavlink2


class MavlinkController:
    """Synchronous-from-caller MAVLink client for PX4 SITL.

    Designed for the Gazebo bridge runtime.  One instance per flight
    session.  Reusing the same instance across sessions is allowed
    (call ``connect()`` again after ``disconnect()``).
    """

    # PX4 requires setpoints to stream at >= 2 Hz for OFFBOARD mode.
    # 20 Hz gives generous headroom and aligns with stock PX4 demos.
    _SETPOINT_RATE_HZ = 20.0

    # We are the ground station, so we must behave like one.  The airframes
    # we generate set NAV_DLL_ACT 2 (data-link-loss -> land), which makes a
    # GCS connection a *precondition for arming*: PX4's rcAndDataLinkCheck
    # blocks every mode with "Preflight Fail: No connection to the GCS"
    # until it has heard a heartbeat from a GCS.  1 Hz is the MAVLink
    # convention and matches PX4's own staleness window.
    _HEARTBEAT_RATE_HZ = 1.0

    def __init__(
        self,
        udp_url: str = "udp:127.0.0.1:14540",
        *,
        source_system: int = 255,
        source_component: int = 0,
        connect_factory: Callable[..., Any] | None = None,
    ) -> None:
        self.udp_url = udp_url
        self.source_system = source_system
        self.source_component = source_component
        self._connect_factory = connect_factory  # for tests

        self._conn: Any = None
        self._target_system: int = 0
        self._target_component: int = 0

        self._mav_lock = threading.RLock()
        self._telemetry = FlightTelemetry()
        self._telemetry_lock = threading.Lock()

        self._setpoint = _Setpoint()
        self._setpoint_lock = threading.Lock()

        self._stop_evt = threading.Event()
        self._rx_thread: threading.Thread | None = None
        self._tx_thread: threading.Thread | None = None
        self._hb_thread: threading.Thread | None = None

        self._ack_events: dict[int, tuple[threading.Event, list[int]]] = {}
        self._ack_lock = threading.Lock()

        self._param_events: dict[str, tuple[threading.Event, list[float]]] = {}
        self._param_lock = threading.Lock()

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    def connect(self, timeout_s: float = 10.0) -> None:
        """Open the MAVLink connection and wait for the first heartbeat.

        Starts the receive thread on success.  Idempotent after a
        successful prior connect — ``disconnect()`` must be called
        between sessions.
        """
        if self._conn is not None:
            return

        if self._connect_factory is None:
            mavutil, _ = _import_mavutil()
            factory = lambda url, ss, sc: mavutil.mavlink_connection(  # noqa: E731
                url,
                source_system=ss,
                source_component=sc,
            )
        else:
            factory = self._connect_factory

        logger.info("Connecting to PX4 MAVLink at %s", self.udp_url)
        conn = factory(self.udp_url, self.source_system, self.source_component)
        msg = conn.wait_heartbeat(timeout=timeout_s)
        if msg is None:
            raise MavlinkError(
                f"Heartbeat timeout after {timeout_s}s on {self.udp_url}",
                code="HEARTBEAT_TIMEOUT",
            )

        self._conn = conn
        self._target_system = conn.target_system
        self._target_component = conn.target_component
        self._update_heartbeat(msg)

        logger.info(
            "MAVLink heartbeat received (sys=%s comp=%s)",
            self._target_system,
            self._target_component,
        )

        self._stop_evt.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_loop,
            name="mavlink-rx",
            daemon=True,
        )
        self._rx_thread.start()

        # Must run for the whole session, not just during OFFBOARD: arming
        # happens before any setpoint stream exists, and it is arming that
        # PX4 gates on seeing a GCS.
        self._hb_thread = threading.Thread(
            target=self._hb_loop,
            name="mavlink-heartbeat",
            daemon=True,
        )
        self._hb_thread.start()

    def disconnect(self) -> None:
        """Stop streamers, close the MAVLink connection, join threads."""
        self._stop_evt.set()
        for t in (self._tx_thread, self._rx_thread, self._hb_thread):
            if t is not None and t.is_alive():
                t.join(timeout=2.0)
        self._tx_thread = None
        self._rx_thread = None
        self._hb_thread = None

        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:  # noqa: BLE001
                pass
            self._conn = None

    def is_connected(self) -> bool:
        if self._conn is None:
            return False
        with self._telemetry_lock:
            last = self._telemetry.last_heartbeat_ts
        # PX4 heartbeats are every 1 s; stale > 5 s is unhealthy.
        return (time.monotonic() - last) < 5.0

    def get_telemetry(self) -> FlightTelemetry:
        with self._telemetry_lock:
            return FlightTelemetry(
                last_heartbeat_ts=self._telemetry.last_heartbeat_ts,
                custom_mode=self._telemetry.custom_mode,
                base_mode=self._telemetry.base_mode,
                armed=self._telemetry.armed,
                local_position=self._telemetry.local_position,
                local_velocity=self._telemetry.local_velocity,
            )

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def arm(self, timeout_s: float = 5.0, *, force: bool = True) -> None:
        """Arm the autopilot.

        ``force`` sends the param2=21196 magic number, but do not rely on it
        to bypass anything over MAVLink: ``Commander.cpp`` calls
        ``arm(reason, cmd.from_external || !forced)``, and ``from_external``
        is true for every command that arrives on a link — so an external
        arm always runs the full preflight gate.  The magic number only
        skips checks for internally-generated commands.

        If arming is denied, read the reason from PX4 rather than guessing:
        ``ulog_messages <newest>.ulg`` prints the ``health_and_arming_checks``
        line that says which check failed.
        """
        self._send_command_long_and_wait(
            command=400,  # MAV_CMD_COMPONENT_ARM_DISARM
            param1=1.0,  # arm
            param2=21196.0 if force else 0.0,
            timeout_s=timeout_s,
            description="arm",
        )

    def disarm(self, timeout_s: float = 5.0) -> None:
        self._send_command_long_and_wait(
            command=400,
            param1=0.0,
            timeout_s=timeout_s,
            description="disarm",
        )

    def takeoff_via_mode(self, altitude_m: float | None = None, timeout_s: float = 5.0) -> None:
        """Trigger AUTO_TAKEOFF mode — same code path as ``commander takeoff``.

        PX4 v1.17 ignores the older MAV_CMD_NAV_TAKEOFF flow without first
        being switched to AUTO_TAKEOFF mode.  This helper does exactly that:
        ``DO_SET_MODE`` to (custom_main=4, custom_sub=2).  Vehicle must
        already be armed.

        AUTO_TAKEOFF carries no altitude of its own — with no mission item to
        read, PX4 climbs to ``MIS_TAKEOFF_ALT`` and logs "Using default takeoff
        altitude: 2.50 m".  So ``altitude_m`` is applied by setting that
        parameter first; without it, asking for 5 m silently gets you 2.5.
        """
        if altitude_m is not None:
            self.set_param("MIS_TAKEOFF_ALT", float(altitude_m), timeout_s=timeout_s)
        self._send_command_long_and_wait(
            command=176,  # MAV_CMD_DO_SET_MODE
            param1=1.0,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            param2=4.0,  # PX4_CUSTOM_MAIN_MODE_AUTO
            param3=2.0,  # PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF
            timeout_s=timeout_s,
            description="set AUTO_TAKEOFF mode",
        )

    def set_param(self, name: str, value: float, timeout_s: float = 5.0) -> float:
        """Set a PX4 parameter and wait for the readback.

        PARAM_SET is not acknowledged by COMMAND_ACK — PX4 replies with a
        PARAM_VALUE carrying whatever it actually stored, which may differ from
        what was asked (clamped to the parameter's range, or rounded for an
        integer parameter).  Returning the stored value rather than the
        requested one keeps callers honest about that.
        """
        if self._conn is None:
            raise MavlinkError(
                f"Cannot set {name!r} — not connected.",
                code="NOT_CONNECTED",
            )

        event = threading.Event()
        values: list[float] = []
        with self._param_lock:
            self._param_events[name] = (event, values)

        try:
            with self._mav_lock:
                self._conn.mav.param_set_send(
                    self._target_system,
                    self._target_component,
                    name.encode("ascii"),
                    float(value),
                    9,  # MAV_PARAM_TYPE_REAL32
                )
            if not event.wait(timeout=timeout_s):
                raise MavlinkError(
                    f"No PARAM_VALUE for {name!r} within {timeout_s}s",
                    code="PARAM_TIMEOUT",
                )
            return values[0]
        finally:
            with self._param_lock:
                self._param_events.pop(name, None)

    def land_via_mode(self, timeout_s: float = 5.0) -> None:
        """Trigger AUTO_LAND mode — same code path as ``commander land``."""
        self._send_command_long_and_wait(
            command=176,  # MAV_CMD_DO_SET_MODE
            param1=1.0,
            param2=4.0,  # AUTO main mode
            param3=6.0,  # AUTO_LAND sub mode
            timeout_s=timeout_s,
            description="set AUTO_LAND mode",
        )

    def takeoff(self, alt_m: float, timeout_s: float = 5.0) -> None:
        """Send MAV_CMD_NAV_TAKEOFF with the requested altitude."""
        self._send_command_long_and_wait(
            command=22,  # MAV_CMD_NAV_TAKEOFF
            param7=float(alt_m),
            timeout_s=timeout_s,
            description=f"takeoff to {alt_m}m",
        )

    def land(self, timeout_s: float = 5.0) -> None:
        self._send_command_long_and_wait(
            command=21,  # MAV_CMD_NAV_LAND
            timeout_s=timeout_s,
            description="land",
        )

    def set_offboard_mode(self, timeout_s: float = 5.0) -> None:
        """Switch PX4 to OFFBOARD.

        PX4 only accepts OFFBOARD when a setpoint stream is already
        flowing at >= 2 Hz, so callers must ``start_setpoint_stream()``
        first and let it run for at least one tick.
        """
        # PX4 OFFBOARD: base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        # custom_mode encodes (main, sub) = (6, 0) -> high word 6, low 0.
        custom_mode = 6 << 24
        self._send_command_long_and_wait(
            command=176,  # MAV_CMD_DO_SET_MODE
            param1=1.0,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            param2=float(custom_mode >> 16),  # PX4 main mode
            param3=0.0,  # PX4 sub mode
            timeout_s=timeout_s,
            description="set OFFBOARD mode",
        )

    def set_velocity(
        self,
        vx_mps: float,
        vy_mps: float,
        vz_mps: float,
        yaw_rate_rps: float,
    ) -> None:
        """Update the streaming setpoint.

        Fire-and-forget: the setpoint thread picks up the new values on
        its next tick.  PX4 will smoothly follow.
        """
        with self._setpoint_lock:
            self._setpoint.vx_mps = float(vx_mps)
            self._setpoint.vy_mps = float(vy_mps)
            self._setpoint.vz_mps = float(vz_mps)
            self._setpoint.yaw_rate_rps = float(yaw_rate_rps)

    # ------------------------------------------------------------------
    # Setpoint stream
    # ------------------------------------------------------------------

    def start_setpoint_stream(self) -> None:
        """Begin streaming velocity setpoints at ``_SETPOINT_RATE_HZ``."""
        if self._tx_thread is not None and self._tx_thread.is_alive():
            return
        if self._conn is None:
            raise MavlinkError(
                "Cannot start setpoint stream — not connected.",
                code="NOT_CONNECTED",
            )
        self._tx_thread = threading.Thread(
            target=self._tx_loop,
            name="mavlink-tx",
            daemon=True,
        )
        self._tx_thread.start()

    def stop_setpoint_stream(self) -> None:
        """Stop the setpoint streamer (without disconnecting)."""
        if self._tx_thread is None:
            return
        # The shared stop_evt also stops rx; only signal the tx thread
        # to exit by joining a timeout-bounded wait.  We piggyback on a
        # separate flag so disconnect's stop_evt doesn't fire here.
        self._tx_stop = True  # type: ignore[attr-defined]
        self._tx_thread.join(timeout=1.5)
        self._tx_thread = None

    # ------------------------------------------------------------------
    # Internal: rx loop
    # ------------------------------------------------------------------

    def _rx_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                msg = self._conn.recv_match(blocking=True, timeout=0.5)
            except Exception as exc:  # noqa: BLE001
                logger.warning("MAVLink recv error: %s", exc)
                time.sleep(0.1)
                continue
            if msg is None:
                continue
            self._handle_message(msg)

    def _handle_message(self, msg: Any) -> None:
        msg_type = msg.get_type()

        if msg_type == "HEARTBEAT":
            self._update_heartbeat(msg)
            return

        if msg_type == "LOCAL_POSITION_NED":
            with self._telemetry_lock:
                self._telemetry.local_position = (
                    float(msg.x),
                    float(msg.y),
                    float(msg.z),
                )
                self._telemetry.local_velocity = (
                    float(msg.vx),
                    float(msg.vy),
                    float(msg.vz),
                )
            return

        if msg_type == "PARAM_VALUE":
            # pymavlink usually decodes param_id, but not always — a char[16]
            # field can arrive as NUL-padded bytes.
            raw = msg.param_id
            name = (raw.decode("ascii", "replace") if isinstance(raw, bytes) else str(raw)).rstrip(
                "\x00"
            )
            with self._param_lock:
                entry = self._param_events.get(name)
            if entry is not None:
                event, values = entry
                values.append(float(msg.param_value))
                event.set()
            return

        if msg_type == "COMMAND_ACK":
            with self._ack_lock:
                entry = self._ack_events.get(int(msg.command))
            if entry is not None:
                event, results = entry
                results.append(int(msg.result))
                event.set()
            return

    def _update_heartbeat(self, msg: Any) -> None:
        # MAV_MODE_FLAG_SAFETY_ARMED == 128
        armed = bool(int(getattr(msg, "base_mode", 0)) & 128)
        with self._telemetry_lock:
            self._telemetry.last_heartbeat_ts = time.monotonic()
            self._telemetry.custom_mode = int(getattr(msg, "custom_mode", 0))
            self._telemetry.base_mode = int(getattr(msg, "base_mode", 0))
            self._telemetry.armed = armed

    # ------------------------------------------------------------------
    # Internal: heartbeat loop
    # ------------------------------------------------------------------

    def _hb_loop(self) -> None:
        """Announce ourselves as a ground station once per second.

        pymavlink does not emit heartbeats on its own — the application
        has to.  Without this PX4 holds ``gcs_connection_lost``, and any
        airframe with ``NAV_DLL_ACT > 0`` refuses to arm indefinitely.
        Note that force-arm does not help: PX4 only honours the 21196
        magic number for internally-generated commands, so a MAVLink arm
        always runs the full preflight gate.
        """
        period = 1.0 / self._HEARTBEAT_RATE_HZ
        while not self._stop_evt.is_set():
            try:
                with self._mav_lock:
                    self._conn.mav.heartbeat_send(
                        6,  # MAV_TYPE_GCS
                        8,  # MAV_AUTOPILOT_INVALID — we are not an autopilot
                        0,  # base_mode
                        0,  # custom_mode
                        4,  # MAV_STATE_ACTIVE
                    )
            except Exception as exc:  # noqa: BLE001
                # A dropped heartbeat is recoverable; a dead connection is
                # the rx loop's business to report.
                logger.warning("MAVLink heartbeat send failed: %s", exc)
            self._stop_evt.wait(period)

    # ------------------------------------------------------------------
    # Internal: tx (setpoint) loop
    # ------------------------------------------------------------------

    def _tx_loop(self) -> None:
        period = 1.0 / self._SETPOINT_RATE_HZ
        # Type mask for SET_POSITION_TARGET_LOCAL_NED: ignore position +
        # acceleration + yaw, use velocity + yaw_rate.  The bitmask
        # values are documented in the MAVLink common.xml.
        # Bits set = ignored. We zero the velocity bits (1,2,4) and the
        # yaw-rate bit (2048), and set everything else.
        type_mask = (
            (1 << 0)
            | (1 << 1)
            | (1 << 2)  # ignore position x/y/z
            # bits 3,4,5 = velocity x/y/z (0 = use)
            | (1 << 6)
            | (1 << 7)
            | (1 << 8)  # ignore acceleration
            | (1 << 10)  # ignore yaw
            # bit 11 = yaw_rate (0 = use)
        )
        # MAV_FRAME_LOCAL_NED == 1
        frame = 1
        self._tx_stop = False  # type: ignore[attr-defined]

        while not self._stop_evt.is_set() and not getattr(self, "_tx_stop", False):
            try:
                with self._setpoint_lock:
                    sp = _Setpoint(
                        vx_mps=self._setpoint.vx_mps,
                        vy_mps=self._setpoint.vy_mps,
                        vz_mps=self._setpoint.vz_mps,
                        yaw_rate_rps=self._setpoint.yaw_rate_rps,
                    )
                with self._mav_lock:
                    self._conn.mav.set_position_target_local_ned_send(
                        0,  # time_boot_ms
                        self._target_system,
                        self._target_component,
                        frame,
                        type_mask,
                        0.0,
                        0.0,
                        0.0,  # x, y, z (ignored)
                        sp.vx_mps,
                        sp.vy_mps,
                        sp.vz_mps,
                        0.0,
                        0.0,
                        0.0,  # afx, afy, afz (ignored)
                        0.0,  # yaw (ignored)
                        sp.yaw_rate_rps,
                    )
            except Exception as exc:  # noqa: BLE001
                logger.warning("MAVLink setpoint send error: %s", exc)
            time.sleep(period)

    # ------------------------------------------------------------------
    # Internal: command_long sender + ACK wait
    # ------------------------------------------------------------------

    def _send_command_long_and_wait(
        self,
        *,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        timeout_s: float = 5.0,
        description: str = "",
    ) -> None:
        if self._conn is None:
            raise MavlinkError(
                f"Cannot send {description!r} — not connected.",
                code="NOT_CONNECTED",
            )

        event = threading.Event()
        results: list[int] = []
        with self._ack_lock:
            self._ack_events[command] = (event, results)

        try:
            with self._mav_lock:
                self._conn.mav.command_long_send(
                    self._target_system,
                    self._target_component,
                    command,
                    0,
                    param1,
                    param2,
                    param3,
                    param4,
                    param5,
                    param6,
                    param7,
                )
            if not event.wait(timeout=timeout_s):
                raise MavlinkError(
                    f"No ACK for {description or command!r} within {timeout_s}s",
                    code="ACK_TIMEOUT",
                )
            # MAV_RESULT_ACCEPTED == 0
            if results and results[0] != 0:
                raise MavlinkError(
                    f"{description or command!r} rejected: result={results[0]}",
                    code="COMMAND_REJECTED",
                )
        finally:
            with self._ack_lock:
                self._ack_events.pop(command, None)
