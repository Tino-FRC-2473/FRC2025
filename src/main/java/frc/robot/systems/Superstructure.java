package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.systems.ClimberFSMSystem.ClimberFSMState;
import frc.robot.systems.DriveFSMSystem.DriveFSMState;
import frc.robot.systems.ElevatorFSMSystem.ElevatorFSMState;
import frc.robot.systems.FunnelFSMSystem.FunnelFSMState;

public class Superstructure {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum SuperFSMState {
		IDLE,
		READY_CORAL,
		PRE_SCORE,
		SCORE_L2,
		SCORE_L3,
		SCORE_L4,
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

			case READY_CORAL:
				handleReadyCoralState(input);
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

				if (funnelSystem.isHoldingCoral()) {
					return SuperFSMState.READY_CORAL;
				}

				return SuperFSMState.IDLE;

			case READY_CORAL:

				if (!funnelSystem.isHoldingCoral()) {
					return SuperFSMState.IDLE;
				}

				return SuperFSMState.READY_CORAL;

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
	 * Handle behavior in READY_CORAL.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleReadyCoralState(TeleopInput input) {
		driveSystem.setState(DriveFSMState.TELEOP_STATE);
		elevatorSystem.setState(ElevatorFSMState.MANUAL);
		funnelSystem.setState(FunnelFSMState.CLOSED);
		climberSystem.setState(ClimberFSMState.IDLE);
	}
}
