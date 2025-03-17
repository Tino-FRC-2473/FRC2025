package frc.robot.systems;

import org.littletonrobotics.junction.Logger;

// WPILib Imports
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
		STOWED,
		EXTEND,
		CLIMB,
		IDLE
	}

	/* ======================== Private variables ======================== */
	private ClimberFSMState currentState;
	private TalonFX climberMotor;

	private DigitalInput climbSwitch;

	private BaseStatusSignal climberPosSignal;

	private double targetPosition;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	/* ======================== Constructor ======================== */
	/**
	 * Create ClimberFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ClimberFSMSystem() {
		// Perform hardware init
		climberMotor = new TalonFXWrapper(
			HardwareMap.CAN_ID_CLIMBER
		);

		var talonFXConfigs = new TalonFXConfiguration();

		var outputConfigs = talonFXConfigs.MotorOutput;
		outputConfigs.NeutralMode = NeutralModeValue.Brake;

		climberMotor.getConfigurator().apply(talonFXConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
			Constants.UPDATE_FREQUENCY_HZ,
			climberMotor.getPosition(),
			climberMotor.getVelocity(),
			climberMotor.getAcceleration(),
			climberMotor.getMotorVoltage());

		climberMotor.optimizeBusUtilization();

		climberPosSignal = climberMotor.getPosition();

		climbSwitch = new DigitalInput(HardwareMap.CLIMBER_LIMIT_SWITCH_DIO_PORT);

		climberMotor.setPosition(Constants.CLIMBER_PID_TARGET_LOW);

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
	 * <p>
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then enabled.
	 */
	public void reset() {
		currentState = ClimberFSMState.IDLE;
		targetPosition = Constants.CLIMBER_PID_TARGET_EXTEND;

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
			case IDLE:
				handleIdleState(input);
				break;

			case STOWED:
				handleAutomaticState(input);
				break;
			case EXTEND:
				handleAutomaticState(input);
				break;
			case CLIMB:
				handleAutomaticState(input);
				break;

			case MANUAL:
				handleManualState(input);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		if (isLimitSwitchPressed()) {
			climberMotor.setPosition(Constants.CLIMBER_ENCODER_RESET_POSITION
				- Constants.CLIMBER_COUNTS_PER_REV);
			if (targetPosition == Constants.CLIMBER_PID_TARGET_LOW
				+ Constants.CLIMBER_COUNTS_PER_REV) {
				targetPosition = Constants.CLIMBER_PID_TARGET_LOW;
			}
		}

		if (!HardwareMap.useSuperStructure()) {
			handleOverrideState(input);
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
		Logger.recordOutput("Climber applied voltage",
			climberMotor.getMotorVoltage().getValueAsDouble());
		Logger.recordOutput("Climber state", currentState.toString());
		Logger.recordOutput("Climber control request", climberMotor.getAppliedControl().toString());
		Logger.recordOutput("Climber switch pressed?", isLimitSwitchPressed());
		Logger.recordOutput("Climber target position", targetPosition);
	}

	/**
	 * Set the state of the FSM.
	 * @param state The state to set the FSM to.
	 */
	public void setState(ClimberFSMState state) {
		switch (state) {
			case STOWED:
				if (climberMotor.getPosition().getValueAsDouble() < 0) {
					targetPosition = Constants.CLIMBER_PID_TARGET_LOW;
				} else {
					targetPosition = Constants.CLIMBER_PID_TARGET_LOW
						+ Constants.CLIMBER_COUNTS_PER_REV;
				}
				break;

			case EXTEND:
				targetPosition = Constants.CLIMBER_PID_TARGET_EXTEND;
				break;

			case CLIMB:
				targetPosition = Constants.CLIMBER_PID_TARGET_CLIMB;
				break;

			default:
				targetPosition = Constants.CLIMBER_PID_TARGET_LOW;
				break;
		}
		currentState = state;
	}

	/**
	 * Handle the climber states under manual superstructure control.
	 * @param input Global TeleopInput if robot in teleop mode or null if
 	 *        the robot is in autonomous mode.
	 */
	public void handleOverrideState(TeleopInput input) {
		currentState = nextState(input);
	}

	/**
	 * Indicates whether the climber position is in the range for being STOWED.
	 * @return if the climber is STOWED
	 */
	public boolean isClimberStowed() {
		return inRange(
			climberMotor.getPosition().getValueAsDouble(),
			Constants.CLIMBER_PID_TARGET_LOW,
			Constants.CLIMBER_PID_MARGIN_OF_ERROR);
	}

	/**
	 * Indicates whether the climber position is in the range for being EXTENDED.
	 * @return if the climber is EXTENDED
	 */
	public boolean isClimberExtended() {
		return inRange(
			climberMotor.getPosition().getValueAsDouble(),
			Constants.CLIMBER_PID_TARGET_EXTEND,
			Constants.CLIMBER_PID_MARGIN_OF_ERROR);
	}

	/**
	 * Indicates whether the climber position is in the range for being CLIMBED.
	 * @return if the climber is CLIMBED
	 */
	public boolean isClimberClimbed() {
		return inRange(
			climberMotor.getPosition().getValueAsDouble(),
			Constants.CLIMBER_PID_TARGET_CLIMB,
			Constants.CLIMBER_PID_MARGIN_OF_ERROR);
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
			case IDLE:
				// copies MANUAL's state transitions
			case MANUAL:
				if (input.isClimbAdvanceStateButtonPressed()) {
					return ClimberFSMState.CLIMB;
				}
				if (input.isClimbManualButtonPressed()) {
					return ClimberFSMState.MANUAL;
				}
				return ClimberFSMState.IDLE;
			case CLIMB: case STOWED: case EXTEND:
				if (input.isClimbManualButtonPressed() || climberPosSignal.getValueAsDouble()
					% Constants.CLIMBER_COUNTS_PER_REV > targetPosition
					|| isLimitSwitchPressed()) {
					return ClimberFSMState.IDLE;
				}
				return ClimberFSMState.CLIMB;
			default:
				throw new UnsupportedOperationException("Invalid State");
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
		if (Robot.isSimulation()) {
			return climberPosSignal.getValueAsDouble()
				> Constants.CLIMBER_ENCODER_RESET_POSITION;
		} else {
			return climbSwitch.get();
		}
	}

	private double calculateTargetPosition() {
		double pos = climberPosSignal.getValueAsDouble();
		if (pos >= Constants.CLIMBER_PID_TARGET_LOW) {
			if (pos >= Constants.CLIMBER_PID_TARGET_EXTEND) {
				if (pos >= Constants.CLIMBER_PID_TARGET_CLIMB) {
					return Constants.CLIMBER_PID_TARGET_LOW
						+ Constants.CLIMBER_COUNTS_PER_REV;
				} else {
					return Constants.CLIMBER_PID_TARGET_CLIMB;
				}
			} else {
				return Constants.CLIMBER_PID_TARGET_EXTEND;
			}
		} else {
			return Constants.CLIMBER_PID_TARGET_LOW;
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in IDLE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *	   the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		climberMotor.set(0);
		targetPosition = calculateTargetPosition();
	}

	/**
	 * Handle behavior in AUTOMATIC.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *	   the robot is in autonomous mode.
	 */
	private void handleAutomaticState(TeleopInput input) {
		if (
			!inRange(
					climberMotor.getPosition().getValueAsDouble(),
					targetPosition,
					Constants.CLIMBER_PID_MARGIN_OF_ERROR)) {
			climberMotor.set(
				targetPosition == Constants.CLIMBER_PID_TARGET_CLIMB
				? Constants.CLIMB_REDUCED_POWER
				: Constants.CLIMB_POWER
			);
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
		climberMotor.set(Constants.CLIMB_POWER);
		targetPosition = calculateTargetPosition();
	}
}
