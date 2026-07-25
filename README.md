# OOMWOO Health Monitor

A small ROS 2 stack-health monitor and software deadman for OOMWOO-compatible
robots.

The monitor checks work-path heartbeats from the critical components expected
for the active task. It emits one MCU-facing heartbeat only while the complete
critical roster is present, fresh, healthy, and past the arming window. Hard
safety reflexes such as cliff, bumper, wheel-drop, and overcurrent handling stay
independent in MCU firmware.

This repository is the maintained home of the prototype accepted in
[makerspet/oomwoo#36](https://github.com/makerspet/oomwoo/pull/36).

## Package

The ROS 2 package is `oomwoo_health_monitor`.

It currently uses JSON over `std_msgs/msg/String` while the OOMWOO message
contract is still evolving:

- `/oomwoo/health/roster`: reliable, transient-local active roster
- `/oomwoo/health/component`: reliable, volatile work-path heartbeats
- `/oomwoo/health/stack`: reliable, transient-local aggregate state
- `/oomwoo/health/mcu_heartbeat`: reliable, volatile live heartbeat

The heartbeat timestamp and monitor must use the same ROS clock. With
`use_sim_time=true`, both follow `/clock`.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select oomwoo_health_monitor
source install/setup.bash
```

## Test

The dependency-free core and ROS adapter tests can run without ROS installed:

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
python3 tools/sim_health_monitor.py
```

To verify the ROS package:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select oomwoo_health_monitor
```

## Run

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch oomwoo_health_monitor health_monitor.launch.py
```

Publish the active roster with compatible transient-local durability:

```bash
ros2 topic pub --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  /oomwoo/health/roster std_msgs/msg/String \
  "{data: '{\"task_id\":\"dock_cycle\",\"components\":[{\"component_id\":\"recovery_safety\",\"critical\":true,\"max_age_sec\":0.5}]}' }"
```

Component heartbeats must be emitted by the component's useful work path. The
producer should calculate `stamp_sec` from its own ROS node clock:

```python
stamp_sec = node.get_clock().now().nanoseconds / 1_000_000_000.0
```

A free-running heartbeat timer is not sufficient because it may continue while
the useful work loop is wedged.

## Documentation

- [Stack watchdog contract](docs/stack_watchdog_contract.md)
- [Recovery-safety integration](docs/recovery_safety_integration.md)

## Vendoring

Consumers should pin a release tag or exact commit rather than tracking `main`.
For a source workspace, add this repository under `src/` and build it with
`colcon`. The installed ROS package includes the contract documents under
`share/oomwoo_health_monitor/docs`.

## License

Apache-2.0.
