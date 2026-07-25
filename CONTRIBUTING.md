# Contributing

Please open an issue before changing the public topic or heartbeat contract.
Safety-related behavior should include a focused regression test.

Run before submitting a pull request:

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
python3 tools/sim_health_monitor.py
python3 -m py_compile oomwoo_health_monitor/*.py tools/sim_health_monitor.py
```

When ROS 2 Jazzy is available, also run:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select oomwoo_health_monitor
```
