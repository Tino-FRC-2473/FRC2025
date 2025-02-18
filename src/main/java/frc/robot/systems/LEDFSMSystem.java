package frc.robot.systems;

import frc.robot.BlinkinLED;

// WPILib Imports

// Third party Hardware Imports

// Robot Imports
import frc.robot.TeleopInput;

public class LEDFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum LEDFSMState {
		REEF_ALIGNING,
		STATION_ALIGNING,
		OFFSET_FROM_TAG,
		YES_CORAL,
		NO_CORAL,
		AUTO,
		CLIMB
	}

	/* ======================== Private variables ======================== */
	private LEDFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	private BlinkinLED led;
	private DriveFSMSystem driveFSMSystem;
	private FunnelFSMSystem funnelFSMSystem;
	private ClimberFSMSystem climberFSMSystem;

	/* ======================== Constructor ======================== */
	/**
	 * Create a FunnelFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 * @param driveSystem the drive FSM.
	 * @param funnelSystem the funnel FSM.
	 * @param climberSystem the climber FSM.
	 */
	public LEDFSMSystem(
		DriveFSMSystem driveSystem,
		FunnelFSMSystem funnelSystem,
		ClimberFSMSystem climberSystem
	) {
		this.driveFSMSystem = driveSystem;
		this.funnelFSMSystem = funnelSystem;
		this.climberFSMSystem = climberSystem;

		// Perform hardware init
		led = new BlinkinLED();

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public LEDFSMState getCurrentState() {
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
		currentState = LEDFSMState.YES_CORAL;

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
		// Handle states
		if (input == null) {
			return;
		}
		switch (currentState) {
			case REEF_ALIGNING:
				handleReefAlignedState(input);
				break;
			case STATION_ALIGNING:
				handleStationAlignedState(input);
				break;
			case OFFSET_FROM_TAG:
				handleOffsetState(input);
				break;
			case YES_CORAL:
				handleYesCoralState(input);
				break;
			case NO_CORAL:
				handleNoCoralState(input);
				break;
			case CLIMB:
				handleClimbState(input);
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		// Switch state
		currentState = nextState(input);
	}

	/**
	 * Run FSM in autonomous mode. This function only calls the FSM state
	 * specific handlers.
	 */
	public void updateAutonomous() {
		handleAutoState();
	}

	/**
	 * Calls all logging and telemetry to be updated periodically.
	 */
	public void updateLogging() {
		// Telemetry and logging
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
	private LEDFSMState nextState(TeleopInput input) {
		//auto --> never called here

		//climb
		if (
			climberFSMSystem.getCurrentState() == ClimberFSMSystem.ClimberFSMState.EXTENDED
				|| climberFSMSystem.getCurrentState() == ClimberFSMSystem.ClimberFSMState.CLIMB
		) {
			return LEDFSMState.CLIMB;
		}

		//align states
		if (driveFSMSystem.getCurrentState()
			== DriveFSMSystem.DriveFSMState.ALIGN_TO_REEF_TAG_STATE) {
			return LEDFSMState.REEF_ALIGNING;
		}

		if (driveFSMSystem.getCurrentState()
			== DriveFSMSystem.DriveFSMState.ALIGN_TO_STATION_TAG_STATE) {
			return LEDFSMState.STATION_ALIGNING;
		}

		//coral states
		if (funnelFSMSystem.isHoldingCoral()) {
			return LEDFSMState.YES_CORAL;
		} else {
			return LEDFSMState.NO_CORAL;
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in REEF_ALIGNING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleReefAlignedState(TeleopInput input) {
		led.setReefAlignColor();
	}

	/**
	 * Handle behavior in STATION_ALIGNING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleStationAlignedState(TeleopInput input) {
		led.setStationAlignedColor();
	}

	/**
	 * Handle behavior in OFFSET_FROM_TAG.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOffsetState(TeleopInput input) {
		led.setOffsetColor();
	}

	/**
	 * Handle behavior in YES_CORAL.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleYesCoralState(TeleopInput input) {
		led.setYesCoralColor();
	}

	/**
	 * Handle behavior in NO_CORAL.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleNoCoralState(TeleopInput input) {
		led.setNoCoralColor();
	}

	/**
	 * Handle behavior in AUTO.
	 */
	private void handleAutoState() {
		led.setAutoColor();
	}

	/**
	 * Handle behavior in CLIMB.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleClimbState(TeleopInput input) {
		led.setClimbColor();
	}
}
