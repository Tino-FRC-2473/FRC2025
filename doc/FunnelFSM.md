# Funnel FSM Spec

```mermaid
---
title: Funnel State Diagram
---
stateDiagram-v2
  state "Outtake: <p> servo open <p> motor stopped" as OUTTAKE
  state "Idle: <p> servo closed <p> motor stopped" as IDLE
  state "Intake: <p> servo closed <p> motor on" as INTAKE

  [*] --> IDLE
  IDLE --> INTAKE: !hasCoral
  INTAKE --> IDLE: hasCoral
  
  IDLE --> OUTTAKE: Outtake button pressed
  OUTTAKE --> IDLE: !Outtake button pressed
```