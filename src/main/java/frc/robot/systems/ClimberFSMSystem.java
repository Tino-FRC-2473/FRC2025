package frc.robot.systems;

import org.littletonrobotics.junction.Logger;

// WPILib Imports
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

// Third party Hardware Imports

// Robot Imports
import frc.robot.constants.Constants;
import frc.robot.logging.MechLogging;
import frc.robot.motors.TalonFXWrapper;
import frc.robot.HardwareMap;
import frc.robot.Robot;
import frc.robot.TeleopInput;

public class ClimberFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum ClimberFSMState {
		MANUAL,
		LOWERED,
		EXTENDED,
		CLIMB
	}

	/* ======================== Private variables ======================== */
	private ClimberFSMState currentState;
	private TalonFX climberMotor;

	private DigitalInput climbSwitch;

	private BaseStatusSignal climberPosSignal;

	private ClimberFSMState previousState;

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

		climberPosSignal = climberMotor.getPosition();

		climbSwitch = new DigitalInput(HardwareMap.CLIMBER_LIMIT_SWITCH_DIO_PORT);

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

		climberMotor.setPosition(Constants.CLIMBER_PID_TARGET_LOW);

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

			case MANUAL:
				handleManualState(input);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);

		// TODO: move this to appropriate state handlers
		if (isLimitSwitchPressed()) {
			climberMotor.setPosition(Constants.CLIMBER_ENCODER_RESET_POSITION);
		}
		MechLogging.getInstance().updatesClimberPose3d(climberMotor.getPosition().getValue());
	}

	/**
	 * Updates the logging for the climber system.
	 */
	public void updateLogging() {
		Logger.recordOutput("Climber encoder absolute",
			climberMotor.getPosition().getValueAsDouble());
		Logger.recordOutput("Climber encoder relative",
			climberMotor.getPosition().getValueAsDouble()
			% Constants.CLIMBER_COUNTS_PER_REV);
		Logger.recordOutput("Climber encoder degrees",
			Units.rotationsToDegrees(climberMotor.getPosition().getValueAsDouble()
			% Constants.CLIMBER_COUNTS_PER_REV
			/ Constants.CLIMBER_COUNTS_PER_REV));
		Logger.recordOutput("Climber velocity", climberMotor.getVelocity().getValueAsDouble());
		Logger.recordOutput("Climber state", currentState.toString());
		Logger.recordOutput("Climber control request", climberMotor.getAppliedControl().toString());
		Logger.recordOutput("Climber switch pressed?", climbSwitch.get());
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
				if (input.isClimbManualButtonPressed()) {
					previousState = ClimberFSMState.LOWERED;
					return ClimberFSMState.MANUAL;
				}
				return ClimberFSMState.LOWERED;

			case EXTENDED:
				if (input.isClimbAdvanceStateButtonPressed()) {
					return ClimberFSMState.CLIMB;
				}
				if (input.isClimbManualButtonPressed()) {
					previousState = ClimberFSMState.EXTENDED;
					return ClimberFSMState.MANUAL;
				}
				return ClimberFSMState.EXTENDED;

			case CLIMB:
				if (input.isClimbAdvanceStateButtonPressed()) {
					return ClimberFSMState.LOWERED;
				}
				if (input.isClimbManualButtonPressed()) {
					previousState = ClimberFSMState.CLIMB;
					return ClimberFSMState.MANUAL;
				}
				return ClimberFSMState.CLIMB;

			case MANUAL:
				if (input.isClimbAdvanceStateButtonPressed()) {
					if (previousState == null) {
						return ClimberFSMState.LOWERED;
					}
					return previousState;
				}
				return ClimberFSMState.MANUAL;

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

	private boolean isLimitSwitchPressed() {
		return Robot.isReal() && climbSwitch.get();
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in LOWERED.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *	   the robot is in autonomous mode.
	 */
	private void handleLoweredState(TeleopInput input) {
		if (climberPosSignal.getValueAsDouble() > 0
			&& !inRange(
			(climberPosSignal.getValueAsDouble() + Constants.CLIMBER_PID_MARGIN_OF_ERROR / 2)
			% Constants.CLIMBER_COUNTS_PER_REV,
			Constants.CLIMBER_PID_TARGET_LOW,
			Constants.CLIMBER_PID_MARGIN_OF_ERROR)
		) {
			System.out.println("REACHED");
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
			(climberPosSignal.getValueAsDouble() + Constants.CLIMBER_PID_MARGIN_OF_ERROR / 2)
			% Constants.CLIMBER_COUNTS_PER_REV,
			Constants.CLIMBER_PID_TARGET_EXTEND,
			Constants.CLIMBER_PID_MARGIN_OF_ERROR)
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
			(climberPosSignal.getValueAsDouble() + Constants.CLIMBER_PID_MARGIN_OF_ERROR / 2)
			% Constants.CLIMBER_COUNTS_PER_REV,
			Constants.CLIMBER_PID_TARGET_CLIMB,
			Constants.CLIMBER_PID_MARGIN_OF_ERROR)
			&& !isLimitSwitchPressed() // stop climbing if limit switch pressed
			// && input.isClimbButtonHeld() // stop climbing if driver lifts button
		) {
			climberMotor.set(Constants.CLIMB_REDUCED_POWER);
		} else {
			climberMotor.set(0);
		}
	}

	/**
	 * Handle behavior in MANUAL.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *	   the robot is in autonomous mode.
	 */
	private void handleManualState(TeleopInput input) {
		// potentially seperate this into idle and manual
		if (input.isClimbManualButtonPressed()) {
			climberMotor.set(Constants.CLIMB_POWER);
		} else {
			climberMotor.set(0);
		}
	}
}
