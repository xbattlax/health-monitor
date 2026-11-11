# Stack Watchdog Contract

The health monitor is the software deadman between ROS2 and the MCU. It does not
replace the MCU's hard reflexes; it decides whether the ROS2 stack is healthy
enough for the MCU to keep accepting commanded motion.

## Component Heartbeat

Topic:

```text
/oomwoo/health/component    std_msgs/msg/String
```

Temporary JSON fields:

- `component_id`: stable component id, for example `recovery_safety`.
- `health`: `ok`, `healthy`, `warn`, `degraded`, or `error`.
- `stamp_sec`: component work timestamp.
- `sequence`: optional monotonic sequence.
- `detail`: optional short diagnostic text.

Rule: publish this from the work path, not from a free-running timer. Examples:

- recovery-safety evaluated a safety/recovery cycle
- Nav2 controller produced a command decision
- localization consumed a scan and produced a pose update
- dock-cycle evaluated a fresh IR/dock state

A timer can keep publishing fresh timestamps while the work loop is stalled.
The monitor cannot distinguish those messages from genuine work progress, so
producers must tie heartbeat emission to completed work. The adapter regression
tests cover both a stopped work-path heartbeat and this contract violation.

### Clock contract

`stamp_sec` MUST use the same ROS clock as the monitor. Producers should derive
it from their node clock:

```python
stamp_sec = node.get_clock().now().nanoseconds / 1_000_000_000.0
```

When `use_sim_time=true`, both producer and monitor therefore follow `/clock`.
Wall-clock timestamps must not be mixed with ROS/simulation timestamps. A
heartbeat timestamp in the future relative to the monitor is treated as stale
and withholds the MCU heartbeat.
If a timer callback runs without ROS time advancing (or after a backward jump),
the monitor reports `clock_stalled`, withholds the MCU heartbeat, and requires a
new arming window once time advances again. If `/clock` is completely paused,
ROS timers may not run at all; the MCU-side watchdog must independently stop on
missing live heartbeats. Slow ROS simulation time is not equivalent to the
MCU's own monotonic watchdog time.

## Roster

Topic:

```text
/oomwoo/health/roster    std_msgs/msg/String
```

QoS expectation:

- `transient_local`
- reliable
- re-published on task transitions

The monitor requests `TRANSIENT_LOCAL`, `RELIABLE`, depth 1. Roster publishers
must offer compatible durability so a monitor started after the active roster
still receives the latest task state.

Temporary JSON fields:

- `task_id`: active task or mode, for example `cleaning_job`.
- `components`: list of expected components.
- each component has `component_id`, `critical`, and `max_age_sec`.

Critical components block MCU heartbeat if missing, stale, or unhealthy.
Advisory components appear in status but do not stop the robot.

## Aggregated Stack State

Topic:

```text
/oomwoo/health/stack    std_msgs/msg/String
```

Temporary JSON fields:

- `state`: `no_roster`, `arming`, `healthy`, `healthy_with_advisory_faults`,
  `fault`, or `clock_stalled`.
- `task_id`: current roster task.
- `emit_mcu_heartbeat`: whether the MCU-facing heartbeat should be emitted.
- `missing_critical`, `stale_critical`, `unhealthy_critical`.
- `advisory_faults`.
- `healthy_for_sec`.
- `source`.

This topic is `TRANSIENT_LOCAL`, `RELIABLE`, depth 1 so diagnostics subscribers
can immediately obtain the current aggregate state.

## MCU Heartbeat

Topic:

```text
/oomwoo/health/mcu_heartbeat    std_msgs/msg/String
```

The MCU-facing bridge should forward this as the stack watchdog heartbeat only
while `emit_mcu_heartbeat=true`.

This topic is deliberately `VOLATILE` and `RELIABLE`. A late subscriber must
wait for a new live heartbeat; replaying a latched, previously healthy heartbeat
would weaken the deadman guarantee. Its history depth is 1 so a stalled
subscriber cannot drain a queue of old healthy beats. Component heartbeats use
depth 10 because multiple components publish to that shared topic.

Fail-safe rules:

- no roster: withhold heartbeat
- arming window not satisfied: withhold heartbeat
- missing critical component: withhold heartbeat
- stale critical component: withhold heartbeat
- unhealthy critical component: withhold heartbeat
- advisory component fault: report but keep heartbeat

The MCU should soft-stop motors on a short heartbeat miss and assert CPU reset
only on a sustained miss, matching the architecture draft.

The first ROS2 adapter in this contribution is
`oomwoo_health_monitor.health_monitor_node`. It consumes the JSON roster and
component heartbeat topics, publishes stack state every 50 ms, and publishes the
MCU heartbeat only while the aggregate state allows it.

## Timing Budget

Choose component freshness so:

```text
component work period < component max_age < MCU soft-stop timeout < CPU reset timeout
```

At vacuum speeds, sub-second detection keeps the travel distance small. The
exact values should be validated under sim-clock jitter and Raspberry Pi load,
because too-tight watchdogs produce false trips.
