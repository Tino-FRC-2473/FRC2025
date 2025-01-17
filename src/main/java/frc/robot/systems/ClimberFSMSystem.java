package frc.robot.systems;

import static edu.wpi.first.units.Units.Revolutions;

// WPILib Imports
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports

// Robot Imports
import frc.robot.constants.Constants;
import frc.robot.motors.TalonFXWrapper;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;
import frc.robot.HardwareMap;
import frc.robot.TeleopInput;

public class ClimberFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum ClimberFSMState {
		LOWERED,
		EXTENDED,
		CLIMB
	}

	private static final double PID_MARGIN_OF_ERROR = 0.01;

	/* ======================== Private variables ======================== */
	private ClimberFSMState currentState;
	private TalonFX climberMotor;
	private final MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0);

	private double currentLoweredPidTarget;
	private double currentExtendedPidTarget;
	private double currentClimbPidTarget;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	/* ======================== Constructor ======================== */
	/**
	 * Create Mech1FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ClimberFSMSystem() {
		// Perform hardware init
		climberMotor = new TalonFXWrapper(HardwareMap.CAN_ID_CLIMBER);
		climberMotor.setNeutralMode(NeutralModeValue.Brake);

		var talonFXConfigs = new TalonFXConfiguration();
		var slot0Configs = talonFXConfigs.Slot0;

		slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
		slot0Configs.kG = Constants.CLIMBER_MM_CONSTANT_G;
		slot0Configs.kS = Constants.CLIMBER_MM_CONSTANT_S;
		slot0Configs.kV = Constants.CLIMBER_MM_CONSTANT_V;
		slot0Configs.kA = Constants.CLIMBER_MM_CONSTANT_A;
		slot0Configs.kP = Constants.CLIMBER_MM_CONSTANT_P;
		slot0Configs.kI = Constants.CLIMBER_MM_CONSTANT_I;
		slot0Configs.kD = Constants.CLIMBER_MM_CONSTANT_D;

		talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.CLIMBER_CONFIG_CONSTANT_CV;
		talonFXConfigs.MotionMagic.MotionMagicAcceleration = Constants.CLIMBER_CONFIG_CONSTANT_A;
		talonFXConfigs.MotionMagic.MotionMagicJerk = Constants.CLIMBER_CONFIG_CONSTANT_J;

		climberMotor.getConfigurator().apply(talonFXConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
			Constants.UPDATE_FREQUENCY_HZ,
			climberMotor.getPosition(),
			climberMotor.getVelocity(),
			climberMotor.getAcceleration(),
			climberMotor.getMotorVoltage());

		climberMotor.optimizeBusUtilization();

		// initialize pid targets
		currentLoweredPidTarget = Constants.CLIMBER_PID_TARGET_LOW;
		currentExtendedPidTarget = Constants.CLIMBER_PID_TARGET_EXTEND;
		currentClimbPidTarget = Constants.CLIMBER_PID_TARGET_CLIMB;

		climberMotor.setPosition(0);


		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public ClimberFSMState getCurrentState() {
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
		currentState = ClimberFSMState.LOWERED;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *	   the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		if (input == null) {
			return;
		}
		switch (currentState) {
			case LOWERED:
				handleLoweredState(input);
				break;

			case EXTENDED:
				handleExtendedState(input);
				break;

			case CLIMB:
				handleClimbState(input);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);

		SmartDashboard.putNumber("Climber encoder", climberMotor.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("Climber velocity", climberMotor.getVelocity().getValueAsDouble());
		SmartDashboard.putString("Climber state", currentState.toString());
		SmartDashboard.putNumber("Climber LOWERED target", currentLoweredPidTarget);
		SmartDashboard.putNumber("Climber EXTENDED target", currentExtendedPidTarget);
		SmartDashboard.putNumber("Climber CLIMB target", currentClimbPidTarget);
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
	 *	   the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private ClimberFSMState nextState(TeleopInput input) {
		switch (currentState) {
			case LOWERED:
				if (input.isClimbAdvanceStateButtonPressed()) {
					return ClimberFSMState.EXTENDED;
				}
				return ClimberFSMState.LOWERED;

			case EXTENDED:
				if (input.isClimbAdvanceStateButtonPressed()) {
					return ClimberFSMState.CLIMB;
				}
				return ClimberFSMState.EXTENDED;

			case CLIMB:
				if (input.isClimbAdvanceStateButtonPressed()) {
					return ClimberFSMState.LOWERED;
				}
				return ClimberFSMState.CLIMB;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in LOWERED.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *	   the robot is in autonomous mode.
	 */
	private void handleLoweredState(TeleopInput input) {
		if (currentLoweredPidTarget < climberMotor.getPosition().getValue().in(Revolutions)
			- Constants.CLIMBER_EXTERNAL_GEAR_RATIO * PID_MARGIN_OF_ERROR) {
			currentLoweredPidTarget += Constants.CLIMBER_EXTERNAL_GEAR_RATIO;
		}
		climberMotor.setControl(mmVoltage.withPosition(currentLoweredPidTarget));
	}

	/**
	 * Handle behavior in EXTENDED.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *	   the robot is in autonomous mode.
	 */
	private void handleExtendedState(TeleopInput input) {
		if (currentExtendedPidTarget < climberMotor.getPosition().getValue().in(Revolutions)
			- Constants.CLIMBER_EXTERNAL_GEAR_RATIO * PID_MARGIN_OF_ERROR) {
			currentExtendedPidTarget += Constants.CLIMBER_EXTERNAL_GEAR_RATIO;
		}
		climberMotor.setControl(mmVoltage.withPosition(currentExtendedPidTarget));
	}

	/**
	 * Handle behavior in CLIMB.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *	   the robot is in autonomous mode.
	 */
	private void handleClimbState(TeleopInput input) {
		if (currentClimbPidTarget < climberMotor.getPosition().getValue().in(Revolutions)
			- Constants.CLIMBER_EXTERNAL_GEAR_RATIO * PID_MARGIN_OF_ERROR) {
			currentClimbPidTarget += Constants.CLIMBER_EXTERNAL_GEAR_RATIO;
		}
		climberMotor.setControl(mmVoltage.withPosition(currentClimbPidTarget));
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
