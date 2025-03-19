package frc.robot.systems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.HardwareMap;

// WPILib Imports

// Third party Hardware Imports

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.constants.LEDConstants;
import frc.robot.systems.ClimberFSMSystem.ClimberFSMState;
import frc.robot.systems.DriveFSMSystem.DriveFSMState;

public class LEDFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum LEDFSMState {
		REEF_ALIGNED,
		STATION_ALIGNED,
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
	private Spark ledController;

	private DriveFSMSystem driveFSMSystem;
	private FunnelFSMSystem funnelFSMSystem;
	private ClimberFSMSystem climberFSMSystem;

	/* ======================== Constructor ======================== */
	/**
	 * Create a LEDFSMSystem and initialize to starting state. Also perform any
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
		ledController = new Spark(HardwareMap.LED_STRIP_PWM_PORT);

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
		currentState = LEDFSMState.NO_CORAL;

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
			case REEF_ALIGNED:
				handleReefAlignedState();
				break;
			case STATION_ALIGNED:
				handleStationAlignedState();
				break;
			case OFFSET_FROM_TAG:
				handleOffsetState();
				break;
			case YES_CORAL:
				handleYesCoralState();
				break;
			case NO_CORAL:
				handleNoCoralState();
				break;
			case CLIMB:
				handleClimbState();
				break;
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
		Logger.recordOutput("LED State", currentState);
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
		if (input == null) {
			return  LEDFSMState.NO_CORAL;
		}
		switch (currentState) {
			case NO_CORAL:
				if (climberFSMSystem.getCurrentState().equals(ClimberFSMState.CLIMB)) {
					return LEDFSMState.CLIMB;
				}

				if (driveFSMSystem.getCurrentState()
					.equals(DriveFSMState.ALIGN_TO_REEF_TAG_STATE)
					|| driveFSMSystem.getCurrentState()
					.equals(DriveFSMState.ALIGN_TO_STATION_TAG_STATE)) {
					return LEDFSMState.OFFSET_FROM_TAG;
				}

				if (funnelFSMSystem.isHoldingCoral()) {
					return LEDFSMState.YES_CORAL;
				}
				return LEDFSMState.NO_CORAL;

			case YES_CORAL:
				if (climberFSMSystem.getCurrentState().equals(ClimberFSMState.CLIMB)) {
					return LEDFSMState.CLIMB;
				}

				if (driveFSMSystem.getCurrentState()
					.equals(DriveFSMState.ALIGN_TO_REEF_TAG_STATE)
					|| driveFSMSystem.getCurrentState()
					.equals(DriveFSMState.ALIGN_TO_STATION_TAG_STATE)) {
					return LEDFSMState.OFFSET_FROM_TAG;
				}

				if (funnelFSMSystem.isHoldingCoral()) {
					return LEDFSMState.YES_CORAL;
				}
				return LEDFSMState.NO_CORAL;

			case CLIMB:
				if (!climberFSMSystem.getCurrentState().equals(ClimberFSMState.CLIMB)) {
					return LEDFSMState.NO_CORAL;
				}
				return LEDFSMState.CLIMB;

			case OFFSET_FROM_TAG:
				if (driveFSMSystem.isAlignedToTag()) {
					if (driveFSMSystem.getCurrentState()
						.equals(DriveFSMState.ALIGN_TO_REEF_TAG_STATE)) {
						return LEDFSMState.REEF_ALIGNED;
					}

					if (driveFSMSystem.getCurrentState()
						.equals(DriveFSMState.ALIGN_TO_STATION_TAG_STATE)) {
						return LEDFSMState.STATION_ALIGNED;
					}
				}

				if (driveFSMSystem.getCurrentState().equals(DriveFSMState.ALIGN_TO_REEF_TAG_STATE)
					|| driveFSMSystem.getCurrentState()
					.equals(DriveFSMState.ALIGN_TO_STATION_TAG_STATE)) {
					return LEDFSMState.OFFSET_FROM_TAG;
				}
				return LEDFSMState.NO_CORAL;

			case REEF_ALIGNED:
				if (driveFSMSystem.getCurrentState()
					.equals(DriveFSMState.ALIGN_TO_REEF_TAG_STATE)) {
					return LEDFSMState.REEF_ALIGNED;
				}
				return LEDFSMState.NO_CORAL;

			case STATION_ALIGNED:
				if (driveFSMSystem.getCurrentState()
					.equals(DriveFSMState.ALIGN_TO_STATION_TAG_STATE)) {
					return LEDFSMState.STATION_ALIGNED;
				}
				return LEDFSMState.NO_CORAL;

			default:
				throw new IllegalStateException("Invalid State: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in REEF_ALIGNED.
	 */
	private void handleReefAlignedState() {
		ledController.set(LEDConstants.LED_GREEN_SOLID);
	}

	/**
	 * Handle behavior in STATION_ALIGNED.
	 */
	private void handleStationAlignedState() {
		ledController.set(LEDConstants.LED_SKY_BLUE_SOLID);
	}

	/**
	 * Handle behavior in OFFSET_FROM_TAG.
	 */
	private void handleOffsetState() {
		ledController.set(LEDConstants.LED_HOT_PINK_SOLID);
	}

	/**
	 * Handle behavior in YES_CORAL.
	 */
	private void handleYesCoralState() {
		ledController.set(LEDConstants.LED_HEARTBEAT_BLUE);
	}

	/**
	 * Handle behavior in NO_CORAL.
	 */
	private void handleNoCoralState() {
		ledController.set(LEDConstants.LED_STROBE_RED);
	}

	/**
	 * Handle behavior in AUTO.
	 */
	private void handleAutoState() {
		ledController.set(LEDConstants.LED_BPM_OCEAN_PALETTE);
	}

	/**
	 * Handle behavior in CLIMB.
	 */
	private void handleClimbState() {
		ledController.set(LEDConstants.LED_COLOR_WAVES_RAINBOW_PALETTE);
	}
}
