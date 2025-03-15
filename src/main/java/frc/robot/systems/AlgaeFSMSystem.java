package frc.robot.systems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import frc.robot.HardwareMap;

// WPILib Imports

// Third party Hardware Imports

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.constants.Constants;
import frc.robot.motors.TalonFXWrapper;

public class AlgaeFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum AlgaeFSMState {
		STOW,
		DEPLOY
	}

	/* ======================== Private variables ======================== */
	private AlgaeFSMState currentState;

	private MotionMagicVoltage motionRequest;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private TalonFX algaeMotor;

	/* ======================== Constructor ======================== */
	/**
	 * Create a AlgaeFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public AlgaeFSMSystem() {
		// Perform hardware init
		algaeMotor = new TalonFXWrapper(HardwareMap.CAN_ID_ALGAE_REMOVER);

		motionRequest = new MotionMagicVoltage(0);

		var talonFXConfigs = new TalonFXConfiguration();

		var outputConfigs = talonFXConfigs.MotorOutput;
		outputConfigs.NeutralMode = NeutralModeValue.Brake;

		var slot0Configs = talonFXConfigs.Slot0;
		slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
		slot0Configs.kG = Constants.ALGAE_KG;
		slot0Configs.kS = Constants.ALGAE_KS;
		slot0Configs.kV = Constants.ALGAE_KV;
		slot0Configs.kA = Constants.ALGAE_KA;
		slot0Configs.kP = Constants.ALGAE_KP;
		slot0Configs.kI = Constants.ALGAE_KI;
		slot0Configs.kD = Constants.ALGAE_KD;
		slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		var motionMagicConfigs = talonFXConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = Constants.ALGAE_CRUISE_VELO;
		motionMagicConfigs.MotionMagicAcceleration = Constants.ALGAE_TARGET_ACCEL;

		algaeMotor.getConfigurator().apply(talonFXConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
				Constants.UPDATE_FREQUENCY_HZ,
				algaeMotor.getPosition(),
				algaeMotor.getVelocity(),
				algaeMotor.getAcceleration(),
				algaeMotor.getMotorVoltage()
		);

		algaeMotor.optimizeBusUtilization();
			// MUST set brake after applying other configs
		algaeMotor.setPosition(0);

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public AlgaeFSMState getCurrentState() {
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
		currentState = AlgaeFSMState.STOW;

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
			case STOW:
				handleStowState(input);
				break;

			case DEPLOY:
				handleDeployState(input);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/**
	 * Calls all logging and telemetry to be updated periodically.
	 */
	public void updateLogging() {
	}

	/**
	 * Set the state of the FSM.
	 * @param state The state to set the FSM to.
	 */
	public void setState(AlgaeFSMState state) {
		currentState = state;
	}

	/**
	 * Handle the funnel states under manual superstructure control.
	 * @param input Global TeleopInput if robot in teleop mode or null if
 	 *        the robot is in autonomous mode.
	 */
	public void handleManualStates(TeleopInput input) {
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
	private AlgaeFSMState nextState(TeleopInput input) {
		switch (currentState) {
			case STOW:
				if (input.isAlgaeButtonPressed()) {
					return AlgaeFSMState.DEPLOY;
				}
				return AlgaeFSMState.STOW;

			case DEPLOY:
				if (input.isAlgaeButtonPressed()) {
					return AlgaeFSMState.STOW;
				}
				return AlgaeFSMState.DEPLOY;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in STOW.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleStowState(TeleopInput input) {
		algaeMotor.setControl(motionRequest.withPosition(0));
	}

	/**
	 * Handle behavior in DEPLOY.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleDeployState(TeleopInput input) {
		algaeMotor.setControl(motionRequest.withPosition(Constants.ALGAE_DEPLOY_POS));
	}
}
