package frc.robot.systems;

// WPILib Imports
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports

// Robot Imports
import frc.robot.constants.Constants;
import frc.robot.logging.MechLogging;
import frc.robot.motors.TalonFXWrapper;
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

	/* ======================== Private variables ======================== */
	private ClimberFSMState currentState;
	private TalonFX climberMotor;

	private double currentLoweredPidTarget;
	private double currentExtendedPidTarget;
	private double currentClimbPidTarget;

	private BaseStatusSignal climberPosSignal;

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

		climberPosSignal = climberMotor.getPosition();


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
		SmartDashboard.putString("Climber control request",
			climberMotor.getAppliedControl().toString());
		
		MechLogging.getInstance().updatesClimberPose3d(climberMotor.getPosition().getValue());
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

	/**
	 * returns if a value is within a margin of a target.
	 * @param value the value.
	 * @param target the target.
	 * @param margin the margin.
	 * @return whether the value is in range of the target.
	 */
	private boolean inRange(double value, double target, double margin) {
		return Math.abs(target - value) <= margin;
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in LOWERED.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *	   the robot is in autonomous mode.
	 */
	private void handleLoweredState(TeleopInput input) {
		if (!inRange(
			climberPosSignal.getValueAsDouble() % Constants.CLIMBER_COUNTS_PER_REV,
			Constants.CLIMBER_PID_TARGET_LOW,
			Constants.CLIMBER_PID_MARGIN_OF_ERROR * Constants.CLIMBER_COUNTS_PER_REV)
		) {
			climberMotor.set(Constants.CLIMB_POWER);
		} else {
			climberMotor.set(0);
		}
	}

	/**
	 * Handle behavior in EXTENDED.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *	   the robot is in autonomous mode.
	 */
	private void handleExtendedState(TeleopInput input) {
		if (!inRange(
			climberPosSignal.getValueAsDouble() % Constants.CLIMBER_COUNTS_PER_REV,
			Constants.CLIMBER_PID_TARGET_EXTEND,
			Constants.CLIMBER_PID_MARGIN_OF_ERROR * Constants.CLIMBER_COUNTS_PER_REV)
		) {
			climberMotor.set(Constants.CLIMB_POWER);
		} else {
			climberMotor.set(0);
		}
	}

	/**
	 * Handle behavior in CLIMB.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *	   the robot is in autonomous mode.
	 */
	private void handleClimbState(TeleopInput input) {
		if (!inRange(
			climberPosSignal.getValueAsDouble() % Constants.CLIMBER_COUNTS_PER_REV,
			Constants.CLIMBER_PID_TARGET_CLIMB,
			Constants.CLIMBER_PID_MARGIN_OF_ERROR * Constants.CLIMBER_COUNTS_PER_REV)
		) {
			climberMotor.set(Constants.CLIMB_POWER);
		} else {
			climberMotor.set(0);
		}
	}
}
