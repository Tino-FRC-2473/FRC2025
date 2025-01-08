# Elevator FSM Spec

```mermaid
---
title: Elevator State Diagram
---
stateDiagram-v2
state "Manual movement" as MANUAL
state "PID to L4" as L4
state "PID to GROUND" as GROUND
state "PID to SOURCE" as SOURCE
[*] --> MANUAL: start

MANUAL --> L4: △ button && !X button && !O button
MANUAL --> GROUND: !△ button && !X button && O button
MANUAL --> SOURCE: !△ button && X button && !O button
L4 --> MANUAL: !△ button || Encoder in range of target
SOURCE --> MANUAL: !Xbutton || Encoder in range of target
GROUND --> MANUAL:  !O button || Encoder in range of target
```