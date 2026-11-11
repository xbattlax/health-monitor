import json
from pathlib import Path
import sys
import types
import unittest
from unittest import mock


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT))


class _String:
    def __init__(self, data=""):
        self.data = data


class _Publisher:
    def __init__(self, qos):
        self.messages = []
        self.qos = qos

    def publish(self, msg):
        self.messages.append(msg)


class _Logger:
    def __init__(self):
        self.warnings = []

    def warn(self, message):
        self.warnings.append(message)


class _Time:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1_000_000_000)


class _Clock:
    def __init__(self, node):
        self._node = node

    def now(self):
        return _Time(self._node.clock_sec)


class _Node:
    def __init__(self, name):
        self.name = name
        self.clock_sec = 0.0
        self.publishers = {}
        self.subscriptions = []
        self.timers = []
        self.logger = _Logger()

    def create_publisher(self, _msg_type, topic, qos):
        publisher = _Publisher(qos)
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions.append((msg_type, topic, callback, qos))

    def create_timer(self, period_sec, callback):
        self.timers.append((period_sec, callback))

    def get_clock(self):
        return _Clock(self)

    def get_logger(self):
        return self.logger

    def destroy_node(self):
        pass


class _ExternalShutdownException(Exception):
    pass


class _ROSInterruptException(Exception):
    pass


class _DurabilityPolicy:
    TRANSIENT_LOCAL = "transient_local"
    VOLATILE = "volatile"


class _HistoryPolicy:
    KEEP_LAST = "keep_last"


class _ReliabilityPolicy:
    RELIABLE = "reliable"


class _QoSProfile:
    def __init__(self, *, history, depth, reliability, durability):
        self.history = history
        self.depth = depth
        self.reliability = reliability
        self.durability = durability


def _ros_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.spin = lambda node: None
    rclpy.ok = lambda: False
    rclpy.shutdown = lambda: None

    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_executors.ExternalShutdownException = _ExternalShutdownException

    rclpy_exceptions = types.ModuleType("rclpy.exceptions")
    rclpy_exceptions.ROSInterruptException = _ROSInterruptException

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = _Node

    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.DurabilityPolicy = _DurabilityPolicy
    rclpy_qos.HistoryPolicy = _HistoryPolicy
    rclpy_qos.QoSProfile = _QoSProfile
    rclpy_qos.ReliabilityPolicy = _ReliabilityPolicy

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = _String

    return {
        "rclpy": rclpy,
        "rclpy.executors": rclpy_executors,
        "rclpy.exceptions": rclpy_exceptions,
        "rclpy.node": rclpy_node,
        "rclpy.qos": rclpy_qos,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }


class HealthMonitorNodeAdapterTest(unittest.TestCase):
    def setUp(self):
        self._module_patch = mock.patch.dict(sys.modules, _ros_stubs())
        self._module_patch.start()
        sys.modules.pop("oomwoo_health_monitor.health_monitor_node", None)

    def tearDown(self):
        self._module_patch.stop()
        sys.modules.pop("oomwoo_health_monitor.health_monitor_node", None)

    def test_node_publishes_stack_state_and_mcu_heartbeat_after_arming(self):
        from oomwoo_health_monitor.health_monitor_node import HealthMonitorNode

        node = HealthMonitorNode()
        node.clock_sec = 10.0
        node._roster_cb(
            _String(
                data=json.dumps(
                    {
                        "task_id": "dock_cycle",
                        "components": [
                            {
                                "component_id": "recovery_safety",
                                "critical": True,
                                "max_age_sec": 1.0,
                            }
                        ],
                    }
                )
            )
        )
        node._component_cb(
            _String(
                data=json.dumps(
                    {
                        "component_id": "recovery_safety",
                        "health": "ok",
                        "stamp_sec": 10.0,
                    }
                )
            )
        )

        node._timer_cb()
        node.clock_sec = 10.3
        node._timer_cb()

        stack_messages = node.publishers["oomwoo/health/stack"].messages
        mcu_messages = node.publishers["oomwoo/health/mcu_heartbeat"].messages
        self.assertEqual(json.loads(stack_messages[0].data)["state"], "arming")
        self.assertEqual(json.loads(stack_messages[-1].data)["state"], "healthy")
        self.assertEqual(len(mcu_messages), 1)
        self.assertEqual(json.loads(mcu_messages[0].data)["task_id"], "dock_cycle")

    def test_qos_latches_roster_and_stack_but_not_mcu_heartbeat(self):
        from oomwoo_health_monitor.health_monitor_node import HealthMonitorNode

        node = HealthMonitorNode()
        roster_subscription = next(
            item for item in node.subscriptions if item[1] == "oomwoo/health/roster"
        )
        component_subscription = next(
            item for item in node.subscriptions if item[1] == "oomwoo/health/component"
        )

        self.assertEqual(
            roster_subscription[3].durability,
            _DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.assertEqual(roster_subscription[3].reliability, _ReliabilityPolicy.RELIABLE)
        self.assertEqual(roster_subscription[3].depth, 1)
        self.assertEqual(
            node.publishers["oomwoo/health/stack"].qos.durability,
            _DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.assertEqual(
            component_subscription[3].durability,
            _DurabilityPolicy.VOLATILE,
        )
        self.assertEqual(component_subscription[3].depth, 10)
        self.assertEqual(
            node.publishers["oomwoo/health/mcu_heartbeat"].qos.durability,
            _DurabilityPolicy.VOLATILE,
        )
        self.assertEqual(
            node.publishers["oomwoo/health/mcu_heartbeat"].qos.depth,
            1,
        )

    def test_stale_critical_component_stops_mcu_heartbeat(self):
        from oomwoo_health_monitor.health_monitor_node import HealthMonitorNode

        node = HealthMonitorNode()
        node.clock_sec = 20.0
        node._roster_cb(
            _String(
                data=json.dumps(
                    {
                        "task_id": "cleaning_job",
                        "components": [
                            {
                                "component_id": "nav2_controller",
                                "critical": True,
                                "max_age_sec": 1.0,
                            }
                        ],
                    }
                )
            )
        )
        node._component_cb(
            _String(
                data=json.dumps(
                    {
                        "component_id": "nav2_controller",
                        "health": "ok",
                        "stamp_sec": 20.0,
                    }
                )
            )
        )

        node.clock_sec = 20.3
        node._timer_cb()
        node.clock_sec = 20.6
        node._timer_cb()
        node.clock_sec = 21.2
        node._timer_cb()

        stack_messages = node.publishers["oomwoo/health/stack"].messages
        self.assertEqual(len(node.publishers["oomwoo/health/mcu_heartbeat"].messages), 1)
        self.assertEqual(json.loads(stack_messages[-1].data)["state"], "fault")
        self.assertEqual(json.loads(stack_messages[-1].data)["stale_critical"], ["nav2_controller"])

    def test_invalid_roster_is_warned_and_ignored(self):
        from oomwoo_health_monitor.health_monitor_node import HealthMonitorNode

        node = HealthMonitorNode()
        node._roster_cb(_String(data="{not json"))

        self.assertEqual(len(node.logger.warnings), 1)
        node._timer_cb()
        stack = json.loads(node.publishers["oomwoo/health/stack"].messages[-1].data)
        self.assertEqual(stack["state"], "no_roster")

    def test_frozen_clock_withholds_heartbeat_and_requires_rearming(self):
        from oomwoo_health_monitor.health_monitor_node import HealthMonitorNode

        node = HealthMonitorNode()
        node.clock_sec = 10.0
        node._roster_cb(_String(data=json.dumps({
            "task_id": "cleaning_job",
            "components": [{"component_id": "nav2", "max_age_sec": 2.0}],
        })))
        node._component_cb(_String(data=json.dumps({
            "component_id": "nav2", "health": "ok", "stamp_sec": 10.0,
        })))
        node._timer_cb()
        node.clock_sec = 10.3
        node._timer_cb()
        heartbeats = node.publishers["oomwoo/health/mcu_heartbeat"].messages
        self.assertEqual(len(heartbeats), 1)

        node._timer_cb()
        self.assertEqual(len(heartbeats), 1)
        stack = json.loads(node.publishers["oomwoo/health/stack"].messages[-1].data)
        self.assertEqual(stack["state"], "clock_stalled")
        self.assertFalse(stack["emit_mcu_heartbeat"])

        node.clock_sec = 10.4
        node._timer_cb()
        self.assertEqual(len(heartbeats), 1)
        node.clock_sec = 10.7
        node._timer_cb()
        self.assertEqual(len(heartbeats), 2)

    def test_work_path_stall_faults_but_timer_only_heartbeat_masks_it(self):
        from oomwoo_health_monitor.health_monitor_node import HealthMonitorNode

        node = HealthMonitorNode()
        node.clock_sec = 20.0
        node._roster_cb(_String(data=json.dumps({
            "task_id": "cleaning_job",
            "components": [{"component_id": "nav2", "max_age_sec": 0.5}],
        })))
        node._component_cb(_String(data=json.dumps({
            "component_id": "nav2", "health": "ok", "stamp_sec": 20.0,
        })))
        node._timer_cb()
        node.clock_sec = 20.3
        node._timer_cb()
        node.clock_sec = 20.6
        node._timer_cb()
        self.assertEqual(len(node.publishers["oomwoo/health/mcu_heartbeat"].messages), 1)
        stack = json.loads(node.publishers["oomwoo/health/stack"].messages[-1].data)
        self.assertEqual(stack["stale_critical"], ["nav2"])

        # A timer that stamps fresh heartbeats without doing work violates the producer contract.
        node._component_cb(_String(data=json.dumps({
            "component_id": "nav2", "health": "ok", "stamp_sec": 20.6,
        })))
        node.clock_sec = 20.7
        node._timer_cb()
        self.assertEqual(len(node.publishers["oomwoo/health/mcu_heartbeat"].messages), 1)
        node.clock_sec = 21.0
        node._timer_cb()
        self.assertEqual(len(node.publishers["oomwoo/health/mcu_heartbeat"].messages), 2)

    def test_backward_clock_jump_withholds_heartbeat(self):
        from oomwoo_health_monitor.health_monitor_node import HealthMonitorNode

        node = HealthMonitorNode()
        node.clock_sec = 10.0
        node._roster_cb(_String(data=json.dumps({
            "task_id": "cleaning_job",
            "components": [{"component_id": "nav2", "max_age_sec": 2.0}],
        })))
        node._component_cb(_String(data=json.dumps({
            "component_id": "nav2", "health": "ok", "stamp_sec": 10.0,
        })))
        node._timer_cb()
        node.clock_sec = 10.3
        node._timer_cb()

        node.clock_sec = 9.0
        node._timer_cb()
        stack = json.loads(node.publishers["oomwoo/health/stack"].messages[-1].data)
        self.assertEqual(stack["state"], "clock_stalled")
        self.assertEqual(len(node.publishers["oomwoo/health/mcu_heartbeat"].messages), 1)


if __name__ == "__main__":
    unittest.main()
