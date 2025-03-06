# Superstructure FSM Spec

```mermaid
---
title: Superstructure State Diagram
---

stateDiagram-v2
state "Idle: <p> Elevator Manual <p> Drive Teleop <p> Funnel Closed <p> Climber Idle" as IDLE
state "Abort: <p> Elevator Stop <p> Drive Teleop <p> Funnel Closed <p> Climber Stop" as ABORT
state "Reset: <p> Elevatoe Ground <p> Drive Teleop <p> Funnel Open <p> Climber Stowed" as RESET

state "Pre Score: <p> Elevator L2 <p> Drive Align to Reef <p> Funnel Closed <p> Climber Idle" as PRE_SCORE

state "Score L2: <p> Elevator L2 <p> Drive Brake <p> Funnel Open <p> Climber Idle" as SCORE_L2
state "Score L3: <p> Elevator L3 <p> Drive Brake <p> Funnel Open <p> Climber Idle" as SCORE_L3
state "Score L4: <p> Elevator L4 <p> Drive Brake <p> Funnel Open <p> Climber Idle" as SCORE_L4

state "Post Score: <p> Elevator Ground <p> Drive Teleop <p> Funnel Closed <p> Climber Idle" as POST_SCORE

state "Pre Climb: <p> Elevator Ground <p> Drive Teleop <p> Funnel Closed <p> Climber Extend" as PRE_CLIMB
state "Climbing: <p> Elevator Ground <p> Drive Creep Forward <p> Funnel Closed <p> Climber to CLIMB Pos " as CLIMBING
state "Reset Climb: <p> Elevator Ground <p> Drive Teleop <p> Funnel Closed <p> Climber Stowed" as RESET_CLIMB

note left of SCORE_L2
Will release coral upon reaching target height
end note

note left of SCORE_L3
Will release coral upon reaching target height
end note

note left of SCORE_L4
Will release coral upon reaching target height
end note

[*] --> IDLE: start 

%% From Idle
IDLE --> PRE_CLIMB : climbButtonPressed && isClimberStowed
IDLE --> CLIMBING : climbButtonPressed && isClimberExtended
IDLE --> RESET_CLIMB : climbButtonPressed && isAtClimbPos
IDLE --> PRE_SCORE: hasCoral && (L2ButtonPressed || L3ButtonPressed || L4ButtonPressed)

%% From Abort
ABORT --> RESET: resetButtonPressed

%% From Reset
RESET --> IDLE: isElevatorAtGround && isClimberStowed

%% From Pre Climb
PRE_CLIMB --> IDLE : isClimberExtended
PRE_CLIMB --> ABORT: abortButtonPressed

%% From Climbing
CLIMBING --> IDLE : isAtClimbPos
CLIMBING --> ABORT: abortButtonPressed

%% From  Reset Climb
RESET_CLIMB --> IDLE : isClimberStowed
RESET_CLIMB --> ABORT: abortButtonPressed


%% From Pre Score
PRE_SCORE --> SCORE_L2: L2ButtonPressed && hasCoral && isDriveAligned
PRE_SCORE --> SCORE_L3: L3ButtonPressed && hasCoral && isDriveAligned
PRE_SCORE --> SCORE_L4: L4ButtonPressed && hasCoral && isDriveAligned
PRE_SCORE --> ABORT: abortButtonPressed

%% From Score L2
SCORE_L2 --> POST_SCORE: !hasCoral && timer > outtakeTimeSecs
SCORE_L2 --> ABORT: abortButtonPressed

%% From Score L3
SCORE_L3 --> POST_SCORE: !hasCoral && timer > outtakeTimeSecs
SCORE_L3 --> ABORT: abortButtonPressed

%% From Score L4
SCORE_L4 --> POST_SCORE: !hasCoral && timer > outtakeTimeSecs
SCORE_L4 --> ABORT: abortButtonPressed

%% From Post Score
POST_SCORE --> IDLE: isElevatorAtGround
POST_SCORE --> ABORT: abortButtonPressed
```