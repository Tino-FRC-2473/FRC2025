package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
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
		INTAKE,
		PRE_L2,
		PRE_L3,
		PRE_L4,
		SCORE,
		RAISE_TO_L2,
		RAISE_TO_L3,
		RAISE_TO_L4,
		POST_SCORE,
		PRE_CLIMB,
		CLIMB,
		RESET_CLIMB,
		ABORT,
		RESET,
		MANUAL
	}

	/* ======================== Private variables ======================== */

	private SuperFSMState currentState;

	private FunnelFSMSystem funnelSystem;
	private ElevatorFSMSystem elevatorSystem;
	private DriveFSMSystem driveSystem;
	private ClimberFSMSystem climberSystem;

	private Timer scoreTimer = new Timer();
	private Timer alignTimer = new Timer();

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

		alignTimer.stop();
		alignTimer.reset();
		scoreTimer.stop();
		scoreTimer.reset();

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
			case IDLE -> handleIdleState(input);
			case INTAKE -> handleIntakeState(input);
			case PRE_L2, PRE_L3, PRE_L4 -> handlePreScoreState(input);
			case RAISE_TO_L2 -> handleRaiseL2State(input);
			case RAISE_TO_L3 -> handleRaiseL3State(input);
			case RAISE_TO_L4 -> handleRaiseL4State(input);
			case SCORE -> handleScoreCoralState(input);
			case POST_SCORE -> handlePostScoreState(input);
			case PRE_CLIMB -> handlePreClimbState(input);
			case CLIMB -> handleClimbState(input);
			case RESET_CLIMB -> handleResetClimbState(input);
			case ABORT -> handleAbortState(input);
			case RESET -> handleResetState(input);
			case MANUAL -> handleManualState(input);
			default -> throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);

		Logger.recordOutput("Super State", currentState);
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
		if (input == null) {
			return SuperFSMState.IDLE;
		}
		if (currentState != SuperFSMState.MANUAL && input.isManualButtonPressed()) {
			return SuperFSMState.MANUAL;
		}
		switch (currentState) {
			case IDLE: if (input.isClimbAdvanceStateButtonPressed()) {
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
				if (funnelSystem.isHoldingCoral() && input.isSuperL2ButtonPressed()) {
					return SuperFSMState.PRE_L2;
				}
				if (funnelSystem.isHoldingCoral() && input.isSuperL3ButtonPressed()) {
					return SuperFSMState.PRE_L3;
				}
				if (funnelSystem.isHoldingCoral() && input.isSuperL4ButtonPressed()) {
					return SuperFSMState.PRE_L4;
				}
				if (!funnelSystem.isHoldingCoral() && input.isSuperIntakeButtonPressed()) {
					return SuperFSMState.INTAKE;
				}
				return SuperFSMState.IDLE;
			case INTAKE: if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				if (!funnelSystem.isHoldingCoral() && input.isSuperIntakeButtonPressed()) {
					return SuperFSMState.INTAKE;
				}
				return SuperFSMState.IDLE;
			case PRE_L2: if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				if ((funnelSystem.isHoldingCoral() && driveSystem.isAlignedToTag()) || alignFin()) {
					return SuperFSMState.RAISE_TO_L2;
				}
				if (driveSystem.canSeeTag()) {
					return SuperFSMState.PRE_L2;
				}
				return SuperFSMState.IDLE;
			case PRE_L3: if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				if ((funnelSystem.isHoldingCoral() && driveSystem.isAlignedToTag()) || alignFin()) {
					return SuperFSMState.RAISE_TO_L3;
				}
				if (driveSystem.canSeeTag()) {
					return SuperFSMState.PRE_L3;
				}
				return SuperFSMState.IDLE;
			case PRE_L4: if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				if (funnelSystem.isHoldingCoral() && driveSystem.isAlignedToTag() || alignFin()) {
					return SuperFSMState.RAISE_TO_L4;
				}
				if (driveSystem.canSeeTag()) {
					return SuperFSMState.PRE_L4;
				}
				return SuperFSMState.IDLE;
			case RAISE_TO_L2:
				if (funnelSystem.isHoldingCoral() && elevatorSystem.isElevatorAtL2()) {
					scoreTimer.restart();
					return SuperFSMState.SCORE;
				}
				if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				return SuperFSMState.RAISE_TO_L2;
			case RAISE_TO_L3:
				if (funnelSystem.isHoldingCoral() && elevatorSystem.isElevatorAtL3()) {
					scoreTimer.restart();
					return SuperFSMState.SCORE;
				}
				if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				return SuperFSMState.RAISE_TO_L3;
			case RAISE_TO_L4:
				if (funnelSystem.isHoldingCoral() && elevatorSystem.isElevatorAtL4()) {
					scoreTimer.restart();
					return SuperFSMState.SCORE;
				}
				if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				return SuperFSMState.RAISE_TO_L4;
			case SCORE:  if ((!funnelSystem.isHoldingCoral())
						&& scoreTimer.get() > Constants.CORAL_SCORE_TIME_SECS) {
					return SuperFSMState.POST_SCORE;
				}
				if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				return SuperFSMState.SCORE;
			case POST_SCORE:  if ((!funnelSystem.isHoldingCoral() || Robot.isSimulation())
						&& elevatorSystem.isElevatorAtGround()) {
					return SuperFSMState.IDLE;
				}
				if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				return SuperFSMState.POST_SCORE;
			case PRE_CLIMB:  if (climberSystem.isClimberExtended()) {
					return SuperFSMState.IDLE;
				}
				if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				return SuperFSMState.PRE_CLIMB;
			case CLIMB:  if (climberSystem.isClimberClimbed()) {
					return SuperFSMState.IDLE;
				}
				if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				return SuperFSMState.CLIMB;
			case RESET_CLIMB:  if (climberSystem.isClimberStowed()) {
					return SuperFSMState.IDLE;
				}
				if (input.isAbortButtonPressed()) {
					return SuperFSMState.ABORT;
				}
				return SuperFSMState.RESET_CLIMB;
			case ABORT:  if (input.isResetButtonPressed()) {
					return SuperFSMState.RESET;
				}
				return SuperFSMState.ABORT;
			case RESET:
				if (climberSystem.isClimberStowed() && elevatorSystem.isElevatorAtGround()) {
					return SuperFSMState.IDLE;
				}
				return SuperFSMState.RESET;
			case MANUAL:  if (input.isManualButtonPressed()) {
					return SuperFSMState.IDLE;
				}
				return SuperFSMState.MANUAL;
			default: throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	private boolean alignFin() {
		alignTimer.start();
		return alignTimer.get() > Constants.ALIGN_TIME_SECS;
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in IDLE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		elevatorSystem.setState(ElevatorFSMState.GROUND);
		funnelSystem.handleOverrideState(input);
		climberSystem.setState(ClimberFSMState.IDLE);
	}

	/**
	 * Handle behavior in PRE_SCORE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handlePreScoreState(TeleopInput input) {
		driveSystem.setState(DriveFSMState.ALIGN_TO_REEF_TAG_STATE);
		elevatorSystem.setState(ElevatorFSMState.LEVEL2);
		funnelSystem.setState(FunnelFSMState.IDLE);
		climberSystem.setState(ClimberFSMState.IDLE);
	}

	/**
	 * Handle behavior in RAISE_L2.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRaiseL2State(TeleopInput input) {
		elevatorSystem.setState(ElevatorFSMState.LEVEL2);
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		funnelSystem.setState(FunnelFSMState.IDLE);
		climberSystem.setState(ClimberFSMState.IDLE);
		alignTimer.stop();
		alignTimer.reset();
	}

	/**
	 * Handle behavior in RAISE_L3.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRaiseL3State(TeleopInput input) {
		elevatorSystem.setState(ElevatorFSMState.LEVEL3);
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		funnelSystem.setState(FunnelFSMState.IDLE);
		climberSystem.setState(ClimberFSMState.IDLE);
		alignTimer.stop();
		alignTimer.reset();
	}

	/**
	 * Handle behavior in RAISE_L4.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRaiseL4State(TeleopInput input) {
		elevatorSystem.setState(ElevatorFSMState.LEVEL4);
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		funnelSystem.setState(FunnelFSMState.IDLE);
		climberSystem.setState(ClimberFSMState.IDLE);
		alignTimer.stop();
		alignTimer.reset();
	}

	/**
	 * Handle behavior in SCORE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleScoreCoralState(TeleopInput input) {
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		elevatorSystem.setState(ElevatorFSMState.MANUAL);
		climberSystem.setState(ClimberFSMState.IDLE);
		funnelSystem.setState(FunnelFSMState.OUTTAKE);
	}

	/**
	 * Handle behavior in POST_SCORE.
	 * @param input Global TeleopInput if robot is in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handlePostScoreState(TeleopInput input) {
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		elevatorSystem.setState(ElevatorFSMState.GROUND);
		funnelSystem.setState(FunnelFSMState.IDLE);
		climberSystem.setState(ClimberFSMState.IDLE);
	}

	/**
	 * Handle behavior in PRE_CLIMB.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handlePreClimbState(TeleopInput input) {
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		elevatorSystem.setState(ElevatorFSMState.GROUND);
		funnelSystem.setState(FunnelFSMState.IDLE);
		climberSystem.setState(ClimberFSMState.EXTEND);
	}

	/**
	 * Handle behavior in CLIMB.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleClimbState(TeleopInput input) {
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		elevatorSystem.setState(ElevatorFSMState.GROUND);
		funnelSystem.setState(FunnelFSMState.IDLE);
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
		funnelSystem.setState(FunnelFSMState.IDLE);
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
		funnelSystem.setState(FunnelFSMState.IDLE);
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
		funnelSystem.setState(FunnelFSMState.IDLE);
		climberSystem.setState(ClimberFSMState.STOWED);
	}

	/**
	 * Handle behavior in INTAKE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 * 	   the robot is in autonomous mode.
	 */
	private void handleIntakeState(TeleopInput input) {
		driveSystem.setState(DriveFSMState.ALIGN_TO_STATION_TAG_STATE);
		elevatorSystem.setState(ElevatorFSMState.MANUAL);
		funnelSystem.setState(FunnelFSMState.IDLE);
		climberSystem.setState(ClimberFSMState.IDLE);
	}

	/**
	 * Handle behavior in MANUAL.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleManualState(TeleopInput input) {
		driveSystem.handleOverrideState(input);
		elevatorSystem.handleOverrideState(input);
		funnelSystem.handleOverrideState(input);
		climberSystem.handleOverrideState(input);
	}
}
