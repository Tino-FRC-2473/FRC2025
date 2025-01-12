# Elevator FSM Spec

```mermaid
---
title: Elevator State Diagram
---
stateDiagram-v2
state "Manual movement: power is set based on right joystick and limited by limit switch" as MANUAL
state "PID to L4: pid target set to L4 height" as L4
state "PID to GROUND: pid target set to the completely unextended position" as GROUND
state "PID to STATION: pid target set to coral station height" as STATION
[*] --> MANUAL: start

MANUAL --> L4: △ button && !X button && !O button
MANUAL --> GROUND: !△ button && !X button && O button
MANUAL --> STATION: !△ button && X button && !O button
L4 --> MANUAL: !△ button
STATION --> MANUAL: !X button
GROUND --> MANUAL:  !O button

note: Limit switch at bottom zeroes encoder and stops downwards movement

```