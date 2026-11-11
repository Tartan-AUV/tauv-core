"""Dependency-free tests of the actual CANDriver methods, using a mocked bus.
Run: python3 test_can_watchdog.py
"""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

class Clock:
    now = 0.0
    def monotonic(self):
        return self.now

class Logger:
    def __getattr__(self, name):
        return lambda *args, **kwargs: None

class Node:
    def get_logger(self):
        return Logger()

clock = Clock()
class Message:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

# Compile the actual class without imports or invoking its hardware constructor.
tree = ast.parse(Path(__file__).with_name("can_driver.py").read_text())
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "CANDriver")
module = ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[]))
namespace = dict(Node=Node, math=math, time=clock, dronecan=NS(
    uavcan=NS(equipment=NS(safety=NS(ArmingStatus=Message), esc=NS(RawCommand=Message)))))
exec(compile(module, "can_driver.py", "exec"), namespace)
CANDriver = namespace["CANDriver"]

class WatchdogIntegration(unittest.TestCase):
    def setUp(self):
        clock.now = 0
        self.d = d = CANDriver.__new__(CANDriver)
        d.esc_count = 2
        d.BIGARM = True
        d.armed = False
        d.throttles = [0, 0]
        d._watchdog_ok = False
        d._last_watchdog_time = None
        d._rearm_ready = False
        d.watchdog_timeout_s = 1.5
        d.watchdog_require_rearm_edge = True
        d.telemetry = {100: {"voltage": 16}}
        d.startup_voltage_min_v = 14
        d.undervoltage_cutoff_v = 12.3
        d.undervoltage_hold_sec = 5
        d._undervoltage_since = None
        d._remap_thrusts = lambda thrust: list(thrust)
        self.bus = []
        d.dronecan_node = NS(broadcast=self.bus.append)

    def heartbeat(self, state="continue"):
        self.d._watchdog_callback(NS(data=state))

    def setpoint(self, armed, thrust=None):
        self.d._thruster_callback(NS(armed=armed, thrust=[50, -40] if thrust is None else thrust))

    def arm(self):
        self.heartbeat()
        self.setpoint(False)
        self.setpoint(True)
        self.assertTrue(self.d.armed)

    def assertStoppedOnWire(self):
        self.d._send_commands()
        self.assertEqual(self.bus[-2].status, 0)
        self.assertEqual(self.bus[-1].cmd, [0, 0])
        self.assertFalse(self.d.armed)

    def test_startup_and_healthy_do_not_arm(self):
        self.setpoint(True)
        self.assertStoppedOnWire()
        self.heartbeat()
        self.setpoint(True)
        self.assertStoppedOnWire()
        self.setpoint(False)
        self.setpoint(True)
        self.d._send_commands()
        self.assertEqual(self.bus[-2].status, 255)
        self.assertEqual(self.bus[-1].cmd, [50, -40])

    def test_kill_recovery_requires_new_arm_edge(self):
        self.arm()
        self.heartbeat("kill")
        self.assertEqual(self.bus[-1].cmd, [0, 0])  # Immediate zero broadcast.
        self.assertTrue(self.d.BIGARM)
        self.setpoint(False)  # Disarm during fault cannot authorize recovery.
        self.heartbeat()
        self.setpoint(True)
        self.assertStoppedOnWire()
        self.setpoint(False)
        self.setpoint(True)
        self.assertTrue(self.d.armed)

    def test_every_nonhealthy_state_stops(self):
        for value in ("kill", "ERROR", "", "garbage", "Continue"):
            with self.subTest(value=value):
                self.arm()
                self.heartbeat(value)
                self.assertStoppedOnWire()

    def test_timeout_at_boundary(self):
        self.arm()
        clock.now = 1.5
        self.assertStoppedOnWire()
        self.heartbeat()
        self.setpoint(True)
        self.assertStoppedOnWire()

    def test_late_heartbeat_cannot_hide_timeout(self):
        self.arm()
        clock.now = 2
        self.heartbeat()
        self.setpoint(True)
        self.assertStoppedOnWire()

    def test_heartbeat_cannot_override_disarm(self):
        self.arm()
        self.setpoint(False)
        self.heartbeat()
        self.assertStoppedOnWire()

    def test_legacy_protocol(self):
        self.heartbeat("OK")
        self.setpoint(False)
        self.setpoint(True)
        self.assertTrue(self.d.armed)
        self.heartbeat("ERROR")
        self.assertStoppedOnWire()

    def test_battery_latch_survives_watchdog_recovery(self):
        self.arm()
        self.d.telemetry[100]["voltage"] = 12.2
        self.d._send_commands()
        clock.now = 5
        self.heartbeat()
        self.d._send_commands()
        self.assertFalse(self.d.BIGARM)
        self.d.telemetry[100]["voltage"] = 16
        self.arm_attempt_after_lockout()
        self.assertStoppedOnWire()

    def arm_attempt_after_lockout(self):
        self.heartbeat()
        self.setpoint(False)
        self.setpoint(True)

    def test_disabled_master_gate_survives_recovery(self):
        self.d.BIGARM = False
        self.arm_attempt_after_lockout()
        self.assertStoppedOnWire()

    def test_default_controller_contract(self):
        self.d.watchdog_require_rearm_edge = False
        self.heartbeat()
        self.assertStoppedOnWire()  # Health alone never arms.
        self.setpoint(True)
        self.assertTrue(self.d.armed)
        self.heartbeat("kill")
        self.assertStoppedOnWire()
        self.heartbeat()
        self.assertStoppedOnWire()  # Stored commands remain cleared.
        self.setpoint(True)         # Existing controller needs no new handshake.
        self.assertTrue(self.d.armed)

    def test_wrong_length_preserves_original_behavior(self):
        self.arm()
        previous = list(self.d.throttles)
        self.setpoint(True, [1])
        self.assertTrue(self.d.armed)
        self.assertEqual(self.d.throttles, previous)

    def test_unrelated_methods_match_original(self):
        original_path = Path(__file__).parent.parent / "upload" / "can_driver.py"
        if not original_path.exists():
            self.skipTest("Original comparison file is not distributed")
        original = ast.parse(original_path.read_text())
        current = ast.parse(Path(__file__).with_name("can_driver.py").read_text())
        def methods(tree):
            return {n.name: n for c in tree.body if isinstance(c, ast.ClassDef)
                    for n in c.body if isinstance(n, ast.FunctionDef)}
        old, new = methods(original), methods(current)
        for name in ("_on_esc_status", "_startup_voltage_ok", "avg_esc_voltage",
                     "_on_node_status", "_remap_thrusts", "_spin_dronecan",
                     "disarm", "destroy_node"):
            self.assertEqual(ast.dump(old[name]), ast.dump(new[name]), name)
        # Only documentation changed in the battery runtime checker.
        old["_check_undervoltage"].body.pop(0)
        new["_check_undervoltage"].body.pop(0)
        self.assertEqual(ast.dump(old["_check_undervoltage"]),
                         ast.dump(new["_check_undervoltage"]))

if __name__ == "__main__":
    unittest.main(verbosity=2)
