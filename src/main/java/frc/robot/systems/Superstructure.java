package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.constants.Constants;
import frc.robot.systems.ClimberFSMSystem.ClimberFSMState;
import frc.robot.systems.DriveFSMSystem.DriveFSMState;
import frc.robot.systems.ElevatorFSMSystem.ElevatorFSMState;
import frc.robot.systems.FunnelFSMSystem.FunnelFSMState;

public class Superstructure {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum SuperFSMState {
		IDLE,
		PRE_SCORE,
		SCORE,
		RAISE_TO_L2,
		RAISE_TO_L3,
		RAISE_TO_L4,
		POST_SCORE,
		PRE_CLIMB,
		CLIMB,
		RESET_CLIMB,
		ABORT,
		RESET
	}

	/* ======================== Private variables ======================== */
	private SuperFSMState currentState;

	private FunnelFSMSystem funnelSystem;
	private ElevatorFSMSystem elevatorSystem;
	private DriveFSMSystem driveSystem;
	private ClimberFSMSystem climberSystem;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 * @param driveFSMSystem
	 * @param funnelFSMSystem
	 * @param elevatorFSMSystem
	 * @param climberFSMSystem
	 */
	public Superstructure(DriveFSMSystem driveFSMSystem, FunnelFSMSystem funnelFSMSystem,
		ElevatorFSMSystem elevatorFSMSystem, ClimberFSMSystem climberFSMSystem) {
		// Perform hardware init
		elevatorSystem = elevatorFSMSystem;
		funnelSystem = funnelFSMSystem;
		driveSystem = driveFSMSystem;
		climberSystem = climberFSMSystem;

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public SuperFSMState getCurrentState() {
		return currentState;
	}
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = SuperFSMState.IDLE;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		switch (currentState) {
			case IDLE:
				handleIdleState(input);
				break;
			case PRE_SCORE:
				handlePreScoreState(input);
				break;
			case RAISE_TO_L3:
				handleRaiseL3State(input);
				break;
			case RAISE_TO_L4:
				handleRaiseL4State(input);
				break;
			case SCORE:
				handleScoreCoralState(input);
				break;
			case PRE_CLIMB:
				handlePreClimbState(input);
				break;
			case CLIMB:
				handleClimbState(input);
				break;
			case RESET_CLIMB:
				handleResetClimbState(input);
				break;
			case ABORT:
				handleAbortState(input);
				break;
			case RESET:
				handleResetState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private SuperFSMState nextState(TeleopInput input) {
		switch (currentState) {
			case IDLE:
				if (input == null) {
					return SuperFSMState.IDLE;
				}
				if (input.isClimbAdvanceStateButtonPressed()) {
					if (climberSystem.isClimberStowed()) {
						return SuperFSMState.PRE_CLIMB;
					}
					if (climberSystem.isClimberExtended()) {
						return SuperFSMState.CLIMB;
					}
					if (climberSystem.isClimberClimbed()) {
						return SuperFSMState.RESET_CLIMB;
					}
				}
				if (funnelSystem.isHoldingCoral()
					&& (input.isL2ButtonPressed()
					|| input.isL3ButtonPressed()
					|| input.isL4ButtonPressed())) {
					return SuperFSMState.PRE_SCORE;
				}
				return SuperFSMState.IDLE;

			case PRE_SCORE:
				if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				if (funnelSystem.isHoldingCoral() && driveSystem.isAlignedToTag()) {
					if (input.isL2ButtonPressed()) {
						return SuperFSMState.RAISE_TO_L2;
					}

					if (input.isL3ButtonPressed()) {
						return SuperFSMState.RAISE_TO_L3;
					}

					if (input.isL4ButtonPressed()) {
						return SuperFSMState.RAISE_TO_L4;
					}
				}
				return SuperFSMState.PRE_SCORE;

			case RAISE_TO_L3:
				if (input == null) {
					return SuperFSMState.IDLE;
				}
				if (funnelSystem.isHoldingCoral() && elevatorSystem.isElevatorAtL3()) {
					return SuperFSMState.SCORE;
				}
				if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				return SuperFSMState.RAISE_TO_L3;

			case RAISE_TO_L4:
				if (input == null) {
					return SuperFSMState.IDLE;
				}
				if (funnelSystem.isHoldingCoral() && elevatorSystem.isElevatorAtL3()) {
					return SuperFSMState.SCORE;
				}
				if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				return SuperFSMState.RAISE_TO_L4;

			case SCORE:
				if (!funnelSystem.isHoldingCoral()
					&& (funnelSystem.getTime() > Constants.CORAL_SCORE_TIME_SECS)) {
					return SuperFSMState.POST_SCORE;
				}
				if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				return SuperFSMState.SCORE;

			case PRE_CLIMB:
				if (climberSystem.isClimberExtended()) {
					return SuperFSMState.IDLE;
				}
				if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				return SuperFSMState.PRE_CLIMB;

			case CLIMB:
				if (climberSystem.isClimberClimbed()) {
					return SuperFSMState.IDLE;
				}
				if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				return SuperFSMState.CLIMB;

			case RESET_CLIMB:
				if (climberSystem.isClimberStowed()) {
					return SuperFSMState.IDLE;
				}
				if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				return SuperFSMState.RESET_CLIMB;

			case ABORT:
				if (input.isResetButtonPressed()) {
					return SuperFSMState.RESET;
				}

				return SuperFSMState.ABORT;

			case RESET:
				if (climberSystem.isClimberStowed() && elevatorSystem.isElevatorAtGround()) {
					return SuperFSMState.IDLE;
				}
				return SuperFSMState.RESET;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in IDLE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		elevatorSystem.setState(ElevatorFSMState.MANUAL);
		funnelSystem.setState(FunnelFSMState.CLOSED);
		climberSystem.setState(ClimberFSMState.IDLE);
	}

	/**
	 * Handle behavior in PRE_SCORE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handlePreScoreState(TeleopInput input) {
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		elevatorSystem.setState(ElevatorFSMState.GROUND);
		funnelSystem.setState(FunnelFSMState.CLOSED);
		climberSystem.setState(ClimberFSMState.IDLE);
	}

	/**
	 * Handle behavior in RAISE_L3.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRaiseL3State(TeleopInput input) {
		elevatorSystem.setState(ElevatorFSMState.LEVEL3);
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		funnelSystem.setState(FunnelFSMState.CLOSED);
		climberSystem.setState(ClimberFSMState.IDLE);
	}

	/**
	 * Handle behavior in RAISE_L4.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRaiseL4State(TeleopInput input) {
		elevatorSystem.setState(ElevatorFSMState.LEVEL4);
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		funnelSystem.setState(FunnelFSMState.CLOSED);
		climberSystem.setState(ClimberFSMState.IDLE);
	}

	/**
	 * Handle behavior in SCORE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleScoreCoralState(TeleopInput input) {
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		elevatorSystem.setState(ElevatorFSMState.LEVEL3);
		climberSystem.setState(ClimberFSMState.IDLE);

		if (elevatorSystem.isElevatorAtL3()) {
			funnelSystem.setState(FunnelFSMState.OUTTAKE);
		} else {
			funnelSystem.setState(FunnelFSMState.CLOSED);
		}
	}

	/**
	 * Handle behavior in PRE_CLIMB.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handlePreClimbState(TeleopInput input) {
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		elevatorSystem.setState(ElevatorFSMState.GROUND);
		funnelSystem.setState(FunnelFSMState.CLOSED);
		climberSystem.setState(ClimberFSMState.EXTEND);
	}

	/**
	 * Handle behavior in CLIMB.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleClimbState(TeleopInput input) {
		driveSystem.setState(DriveFSMState.TELEOP_STATE); // TODO: change to drive creep forwards
		elevatorSystem.setState(ElevatorFSMState.GROUND);
		funnelSystem.setState(FunnelFSMState.CLOSED);
		climberSystem.setState(ClimberFSMState.CLIMB);
	}

	/**
	 * Handle behavior in RESET_CLIMB.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleResetClimbState(TeleopInput input) {
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		elevatorSystem.setState(ElevatorFSMState.GROUND);
		funnelSystem.setState(FunnelFSMState.CLOSED);
		climberSystem.setState(ClimberFSMState.STOWED);
	}

	/**
	 * Handle behavior in ABORT.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleAbortState(TeleopInput input) {
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		elevatorSystem.setState(ElevatorFSMState.MANUAL);
		funnelSystem.setState(FunnelFSMState.CLOSED);
		climberSystem.setState(ClimberFSMState.IDLE);
	}

	/**
	 * Handle behavior in RESET.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleResetState(TeleopInput input) {
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		elevatorSystem.setState(ElevatorFSMState.GROUND);
		funnelSystem.setState(FunnelFSMState.CLOSED); // TODO: confirm funnel reset pos
		climberSystem.setState(ClimberFSMState.STOWED);
	}
}
