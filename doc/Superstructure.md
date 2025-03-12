# Superstructure FSM Spec

```mermaid
---
title: Superstructure State Diagram
---

stateDiagram-v2

state "Idle: <p> Elevator Manual <p> Drive Teleop <p> Funnel Closed <p> Climber Idle" as IDLE
state "Abort: <p> Elevator Stop <p> Drive Teleop <p> Funnel Closed <p> Climber Stop" as ABORT
state "Manual: <p> Elevator Manual <p> Drive Teleop <p> Funnel Manual <p> Climber Manual" as MANUAL
state "Reset: <p> Elevator Ground <p> Drive Teleop <p> Funnel Open <p> Climber Stowed" as RESET

state "Has Coral: <p> Elevator Ground <p> Drive Teleop <p> Funnel Closed <p> Climber Idle" as HAS_CORAL
state "Pre Score: <p> Elevator Ground <p> Drive Align to Reef <p> Funnel Closed <p> Climber Idle" as PRE_SCORE

state "Raise to L2: <p> Elevator L2 <p> Drive Brake <p> Funnel Closed <p> Climber Idle" as RAISE_L2
state "Raise to L3: <p> Elevator L3 <p> Drive Brake <p> Funnel Closed <p> Climber Idle" as RAISE_L3
state "Raise to L4: <p> Elevator L4 <p> Drive Brake <p> Funnel Closed <p> Climber Idle" as RAISE_L4

state "Score Coral: <p> Elevator Manual <p> Drive Brake <p> Funnel Open <p> Climber Idle" as SCORE

state "Post Score: <p> Elevator Ground <p> Drive Teleop <p> Funnel Closed <p> Climber Idle" as POST_SCORE

state "Pre Climb: <p> Elevator Ground <p> Drive Teleop <p> Funnel Closed <p> Climber Extend" as PRE_CLIMB
state "Climbing: <p> Elevator Ground <p> Drive Creep Forward <p> Funnel Closed <p> Climber to CLIMB Pos " as CLIMBING
state "Reset Climb: <p> Elevator Ground <p> Drive Teleop <p> Funnel Closed <p> Climber Stowed" as RESET_CLIMB

%% fork and join states
state idle_fork <<fork>>
state idle_join <<join>>

state score_fork <<fork>>
state score_join <<join>>

state manual_join <<join>>
state abort_join <<join>>

[*] --> IDLE: start 

%% From Idle (idle fork)
IDLE --> idle_fork
idle_join --> IDLE

idle_fork --> PRE_CLIMB : climbButtonPressed && isClimberStowed
idle_fork --> CLIMBING : climbButtonPressed && isClimberExtended
idle_fork --> RESET_CLIMB : climbButtonPressed && isAtClimbPos
idle_fork --> HAS_CORAL: hasCoral
idle_fork --> manual_join: manualButtonPressed

state Climb {
	%% From Pre Climb
	PRE_CLIMB --> idle_join : isClimberExtended
	PRE_CLIMB --> abort_join: abortButtonPressed
	PRE_CLIMB --> manual_join: manualButtonPressed

	%% From Climbing
	CLIMBING --> idle_join : isAtClimbPos
	CLIMBING --> abort_join: abortButtonPressed
	CLIMBING --> manual_join: manualButtonPressed

	%% From  Reset Climb
	RESET_CLIMB --> idle_join : isClimberStowed
	RESET_CLIMB --> abort_join: abortButtonPressed
	RESET_CLIMB --> manual_join: manualButtonPressed
}

state Score {
	%% From Has Coral
	HAS_CORAL --> PRE_SCORE: hasCoral && (L2ButtonPressed || L3ButtonPressed || L4ButtonPressed)
	HAS_CORAL --> idle_join: !hasCoral
	HAS_CORAL --> manual_join: manualButtonPressed

	%% From Pre Score
	PRE_SCORE --> score_fork
	score_fork --> RAISE_L2: L2ButtonPressed && hasCoral && isDriveAligned
	score_fork --> RAISE_L3: L3ButtonPressed && hasCoral && isDriveAligned
	score_fork --> RAISE_L4: L4ButtonPressed && hasCoral && isDriveAligned
	PRE_SCORE --> abort_join: abortButtonPressed
	PRE_SCORE --> manual_join: manualButtonPressed

	score_join --> SCORE

	%% From Raise L2
	RAISE_L2 --> score_join: hasCoral && isElevatorAtL2
	RAISE_L2 --> abort_join: abortButtonPressed
	RAISE_L2 --> manual_join: manualButtonPressed

	%% From Raise L3
	RAISE_L3 --> score_join: hasCoral  && isElevatorAtL3
	RAISE_L3 --> abort_join: abortButtonPressed
	RAISE_L3 --> manual_join: manualButtonPressed

	%% From Raise L4
	RAISE_L4 --> score_join: hasCoral  && isElevatorAtL4
	RAISE_L4 --> abort_join: abortButtonPressed
	RAISE_L4 --> manual_join: manualButtonPressed

	%% From Score
	SCORE --> POST_SCORE:timer > outtakeTimeSecs
	SCORE --> abort_join: abortButtonPressed
	SCORE --> manual_join: manualButtonPressed

	%% From Post Score
	POST_SCORE --> idle_join: isElevatorAtGround
	POST_SCORE --> abort_join: abortButtonPressed
	POST_SCORE --> manual_join: manualButtonPressed
}

state Overrides {
	%% From Manual
	manual_join --> MANUAL
	MANUAL --> idle_join: manualButtonPressed

	%% From Abort
	abort_join --> ABORT
	ABORT --> RESET: resetButtonPressed

	%% From Reset
	RESET --> idle_join: isElevatorAtGround && isClimberStowed
}