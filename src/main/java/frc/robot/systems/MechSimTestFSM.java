package frc.robot.systems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// WPILib Imports

// Third party Hardware Imports

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.constants.Constants;
import frc.robot.motors.TalonFXWrapper;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

public class MechSimTestFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		ONE,
		TWO,
		MANUAL
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	private TalonFX testMotor;
	private final MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0);

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	/* ======================== Constructor ======================== */
	/**
	 * Create Mech1FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public MechSimTestFSM() {
		// Perform hardware init
		testMotor = new TalonFXWrapper(2);

		testMotor.setNeutralMode(NeutralModeValue.Brake);

		var talonFXConfigs = new TalonFXConfiguration();
		// set slot 0 gains
		var slot0Configs = talonFXConfigs.Slot0;
		slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
		// note: detail on constants in Constants.java
		slot0Configs.kG = Constants.ELEVATOR_MM_CONSTANT_G;
		slot0Configs.kS = Constants.ELEVATOR_MM_CONSTANT_S;
		slot0Configs.kV = Constants.ELEVATOR_MM_CONSTANT_V;
		slot0Configs.kA = Constants.ELEVATOR_MM_CONSTANT_A;
		slot0Configs.kP = Constants.ELEVATOR_MM_CONSTANT_P;
		slot0Configs.kI = Constants.ELEVATOR_MM_CONSTANT_I;
		slot0Configs.kD = Constants.ELEVATOR_MM_CONSTANT_D;

		// set Motion Magic settings
		var motionMagicConfigs = talonFXConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = Constants.ELEVATOR_CONFIG_CONSTANT_CV;
		motionMagicConfigs.MotionMagicAcceleration = Constants.ELEVATOR_CONFIG_CONSTANT_A;
		motionMagicConfigs.MotionMagicJerk = Constants.ELEVATOR_CONFIG_CONSTANT_J;

		testMotor.getConfigurator().apply(talonFXConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
			Constants.UPDATE_FREQUENCY_HZ,
			testMotor.getPosition(),
			testMotor.getVelocity(),
			testMotor.getAcceleration(),
			testMotor.getMotorVoltage());

		testMotor.optimizeBusUtilization();


		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
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
		currentState = FSMState.ONE;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		switch (currentState) {
			case ONE:
				handleOneState(input);
				break;

			case TWO:
				handleTwoState(input);
				break;

			case MANUAL:
				handleManualState(input);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);

		// Logging

		SmartDashboard.putNumber("Test Position",
			testMotor.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("Test Velocity",
			testMotor.getVelocity().getValueAsDouble());

		SmartDashboard.putString("Test State", currentState.toString());
	}

	/**
	 * Performs specific action based on the autoState passed in.
	 * @param autoState autoState that the subsystem executes.
	 * @return if the action carried out in this state has finished executing
	 */
	public boolean updateAutonomous(AutoFSMState autoState) {
		switch (autoState) {
			case STATE1:
				return handleAutoState1();
			case STATE2:
				return handleAutoState2();
			case STATE3:
				return handleAutoState3();
			default:
				return true;
		}
	}

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case ONE:
				if (input != null && input.getDriveLeftJoystickX() > 0.1) {
					return FSMState.TWO;
				}
				if (input != null && input.getDriveLeftJoystickY() > 0.1) {
					return FSMState.MANUAL;
				}
				return FSMState.ONE;

			case TWO:
				if (input != null && input.getDriveLeftJoystickX() < 0.1) {
					return FSMState.ONE;
				}
				if (input != null && input.getDriveLeftJoystickY() > 0.1) {
					return FSMState.MANUAL;
				}
				return FSMState.TWO;

			case MANUAL:
				if (input != null && input.getDriveLeftJoystickY() < 0.1) {
					return FSMState.ONE;
				}
				return FSMState.MANUAL;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in ONE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	private void handleOneState(TeleopInput input) {
		testMotor.setControl(mmVoltage.withPosition(10));
	}
	/**
	 * Handle behavior in TWO.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *		the robot is in autonomous mode.
	 */
	private void handleTwoState(TeleopInput input) {
		testMotor.setControl(mmVoltage.withPosition(100));
	}

	private void handleManualState(TeleopInput input) {
		double signalInput = input.getDriveRightJoystickX();
		testMotor.set(signalInput);
	}

	/**
	 * Performs action for auto STATE1.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoState1() {
		return true;
	}

	/**
	 * Performs action for auto STATE2.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoState2() {
		return true;
	}

	/**
	 * Performs action for auto STATE3.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoState3() {
		return true;
	}
}
