# Climber FSM Spec

```mermaid
---
title: Climber State Diagram
---
stateDiagram-v2
state "PID to LOWERED: target set to completely unextended position, within robot frame" as LOWERED
state "PID to EXTENDED: target set to 90 degrees" as EXTENDED
state "PID to CLIMB: target set to (TBD) degrees from the horizontal" as CLIMB
[*] --> LOWERED: start


LOWERED --> EXTENDED: advance button pressed
LOWERED --> CLIMB: regress button pressed
EXTENDED --> CLIMB: advance button pressed
EXTENDED --> LOWERED: regress button pressed
CLIMB --> EXTENDED: advance button pressed
CLIMB --> LOWERED: regress button pressed
```

