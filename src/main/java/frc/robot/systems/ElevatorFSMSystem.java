package frc.robot.systems;


// WPILib Imports

// Third party Hardware Imports
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HardwareMap;
import frc.robot.Robot;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.constants.Constants;
import frc.robot.logging.MechLogging;
import frc.robot.motors.TalonFXWrapper;


public class ElevatorFSMSystem {

	/* ======================== Constants ======================== */

	// FSM state definitions
	public enum ElevatorFSMState {
		MANUAL,
		GROUND,
		LEVEL2,
		LEVEL3,
		LEVEL4
	}

	/* ======================== Private variables ======================== */

	private ElevatorFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private TalonFX elevatorMotor;
	private DigitalInput groundLimitSwitch;
	private DigitalInput topLimitSwitch;

	/* ======================== Constructor ======================== */

	/**
	 * Create ElevatorFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ElevatorFSMSystem() {
		// Perform hardware init
		elevatorMotor = new TalonFXWrapper(HardwareMap.CAN_ID_ELEVATOR);

		var talonFXConfigs = new TalonFXConfiguration();

		// apply sw limit
		var swLimitSwitch = talonFXConfigs.SoftwareLimitSwitch;
		swLimitSwitch.ForwardSoftLimitEnable = true; // enable top limit
		swLimitSwitch.ReverseSoftLimitEnable = true; // enable bottom limit
		swLimitSwitch.ForwardSoftLimitThreshold = Constants.ELEVATOR_UPPER_THRESHOLD;
		swLimitSwitch.ReverseSoftLimitThreshold = 0;

		elevatorMotor.getConfigurator().apply(talonFXConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
				Constants.UPDATE_FREQUENCY_HZ,
				elevatorMotor.getPosition(),
				elevatorMotor.getVelocity(),
				elevatorMotor.getAcceleration(),
				elevatorMotor.getMotorVoltage()
		);

		elevatorMotor.optimizeBusUtilization();

		elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
			// MUST set brake after applying other configs

		// Initialize limit switches
		groundLimitSwitch = new DigitalInput(HardwareMap.ELEVATOR_GROUND_LIMIT_SWITCH_DIO_PORT);
			//okay for stopping and resetting
		topLimitSwitch = new DigitalInput(HardwareMap.ELEVATOR_TOP_LIMIT_SWITCH_DIO_PORT);
			//only use to stop, DO NOT USE TO RESET

		// Reset state machine

		elevatorMotor.setPosition(Constants.ELEVATOR_TARGET_GROUND);

		reset();
	}

	/* ======================== Public methods ======================== */

	/**
	 * Get the current FSM state.
	 * @return current FSM state
	 */
	public ElevatorFSMState getCurrentState() {
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
		currentState = ElevatorFSMState.MANUAL;

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

		if (input == null) {
			return;
		}
		switch (currentState) {
			case MANUAL:
				handleManualState(input);
				break;
			case GROUND:
				handleGroundState(input);
				break;
			case LEVEL2:
				handleL2State(input);
				break;
			case LEVEL3:
				handleL3State(input);
				break;
			case LEVEL4:
				handleL4State(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);

		// telemetry and logging

		SmartDashboard.putNumber("Elevator encoder",
			elevatorMotor.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("Elevator velocity",
			elevatorMotor.getVelocity().getValueAsDouble());

		SmartDashboard.putBoolean("Elevator bottom limit switch pressed", isBottomLimitReached());
		SmartDashboard.putBoolean("Elevator top limit switch reached", isTopLimitReached());

		SmartDashboard.putString("Elevator State", currentState.toString());
		SmartDashboard.putNumber("Elevator Voltage",
			elevatorMotor.getMotorVoltage().getValueAsDouble());

		SmartDashboard.putNumber("Elevator Accel",
			elevatorMotor.getAcceleration().getValueAsDouble());

		MechLogging.getInstance().updateElevatorPose3d(elevatorMotor.getPosition().getValue());

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
	private ElevatorFSMState nextState(TeleopInput input) {
		if (input == null) {
			return ElevatorFSMState.MANUAL;
		}
		switch (currentState) {
			case MANUAL:
				if (input.isGroundButtonPressed()
					&& !isBottomLimitReached()
					&& !input.isL4ButtonPressed()
					&& !input.isL2ButtonPressed()
					&& !input.isL3ButtonPressed()) {
					return ElevatorFSMState.GROUND;
				}
				if (input.isL4ButtonPressed()
					&& !isTopLimitReached()
					&& !input.isGroundButtonPressed()
					&& !input.isL2ButtonPressed()
					&& !input.isL3ButtonPressed()) {
					return ElevatorFSMState.LEVEL4;
				}
				if (input.isL2ButtonPressed()
					&& Math.abs(elevatorMotor.getPosition().getValueAsDouble()
					- Constants.ELEVATOR_TARGET_L2)
					>= Constants.ELEVATOR_TARGET_MARGIN
					&& !input.isL4ButtonPressed()
					&& !input.isGroundButtonPressed()
					&& !input.isL3ButtonPressed()) {
					return ElevatorFSMState.LEVEL2;
				}
				if (input.isL3ButtonPressed()
					&& Math.abs(elevatorMotor.getPosition().getValueAsDouble()
					- Constants.ELEVATOR_TARGET_L3)
					>= Constants.ELEVATOR_TARGET_MARGIN
					&& !input.isL4ButtonPressed()
					&& !input.isGroundButtonPressed()
					&& !input.isL2ButtonPressed()) {
					return ElevatorFSMState.LEVEL3;
				}
				return ElevatorFSMState.MANUAL;

			case GROUND:
				if (isBottomLimitReached() || !input.isGroundButtonPressed()) {
					return ElevatorFSMState.MANUAL;
				}
				return ElevatorFSMState.GROUND;

			case LEVEL2:
				if (Math.abs(elevatorMotor.getPosition().getValueAsDouble()
					- Constants.ELEVATOR_TARGET_L2)
					< Constants.ELEVATOR_TARGET_MARGIN
					|| !input.isL2ButtonPressed()) {
					return ElevatorFSMState.MANUAL;
				}
				return ElevatorFSMState.LEVEL2;

			case LEVEL3:
				if (Math.abs(elevatorMotor.getPosition().getValueAsDouble()
					- Constants.ELEVATOR_TARGET_L3)
					< Constants.ELEVATOR_TARGET_MARGIN
					|| !input.isL3ButtonPressed()) {
					return ElevatorFSMState.MANUAL;
				}
				return ElevatorFSMState.LEVEL3;

			case LEVEL4:
				if (isTopLimitReached() || !input.isL4ButtonPressed()) {
					return ElevatorFSMState.MANUAL;
				}
				return ElevatorFSMState.LEVEL4;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/**
	 * Getter for the result of the elevator's bottom limit switch.
	 * @return whether the limit is reached
	 */
	private boolean isBottomLimitReached() {
		if (Robot.isSimulation()) {
			return false;
		}
		return groundLimitSwitch.get(); // switch is normally open
	}

	/**
	 * Getter for the result of the elevator's top limit switch.
	 * @return whether the limit is reached
	 */
	private boolean isTopLimitReached() {
		if (Robot.isSimulation()) {
			return false;
		}
		// return !topLimitSwitch.get(); // switch is normally closed
		return false; // temp disable
	}

	/* ------------------------ FSM state handlers ------------------------ */

	/**
	 * Handle behavior in MANUAL.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleManualState(TeleopInput input) {
		double signalInput = input.getManualElevatorMovementInput();
		signalInput = MathUtil.applyDeadband(signalInput, Constants.ELEVATOR_DEADBAND);
		if (isBottomLimitReached()) {
			elevatorMotor.setPosition(Constants.ELEVATOR_TARGET_GROUND);
			if (signalInput < 0) {
				elevatorMotor.set(0); //don't go even further down if you hit the lower limit!
				return;
			}
		}

		if (isTopLimitReached()) {
			if (signalInput > 0) {
				elevatorMotor.set(0); //don't go even further up if you hit the upper limit!
				return;
			}
		}

		elevatorMotor.set(signalInput * Constants.ELEVATOR_MANUAL_SCALE);
	}

	/**
	 * Handle behavior in GROUND.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleGroundState(TeleopInput input) {
		if (isBottomLimitReached()) {
			elevatorMotor.set(0);
			elevatorMotor.setPosition(Constants.ELEVATOR_TARGET_GROUND);
		} else {
			elevatorMotor.set(-Constants.ELEVATOR_POWER);
		}
	}

	/**
	 * Handle behavior in L2.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleL2State(TeleopInput input) {
		if (elevatorMotor.getPosition().getValueAsDouble()
			< Constants.ELEVATOR_TARGET_L2) {
			elevatorMotor.set(Constants.ELEVATOR_POWER);
		} else if (elevatorMotor.getPosition().getValueAsDouble()
			> Constants.ELEVATOR_TARGET_L2) {
			elevatorMotor.set(-Constants.ELEVATOR_POWER);
		}
	}

	/**
	 * Handle behavior in L3.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleL3State(TeleopInput input) {
		if (elevatorMotor.getPosition().getValueAsDouble()
			< Constants.ELEVATOR_TARGET_L3) {
			elevatorMotor.set(Constants.ELEVATOR_POWER);
		} else if (elevatorMotor.getPosition().getValueAsDouble()
			> Constants.ELEVATOR_TARGET_L3) {
			elevatorMotor.set(-Constants.ELEVATOR_POWER);
		}
	}

	/**
	 * Handle behavior in L4.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleL4State(TeleopInput input) {
		if (isTopLimitReached()) {
			elevatorMotor.set(0);
		} else {
			if (Constants.ELEVATOR_TARGET_L4
				- elevatorMotor.getPosition().getValueAsDouble()
				< Constants.ELEVATOR_SPEED_REDUCTION_THRESHOLD_SIZE) {
				elevatorMotor.set(Constants.ELEVATOR_REDUCED_POWER);
			} else {
				elevatorMotor.set(Constants.ELEVATOR_POWER);
			}
		}
	}

	/* ---- Elevator Commands ---- */

	/** Superclass for elevator commands. */
	abstract class ElevatorCommand extends Command {
		private double target;

		ElevatorCommand() { }

		@Override
		public void execute() { }

		@Override
		public boolean isFinished() {
			return Math.abs(elevatorMotor.getPosition().getValueAsDouble()
			- target) < Constants.ELEVATOR_DEADBAND;
		}

		@Override
		public void end(boolean interrupted) { }

		protected void setTarget(double newTarget) {
			this.target = newTarget;
		}

		protected double getTarget() {
			return this.target;
		}
	}

	/** A command that moves the elevator to the Ground position. */
	class ElevatorGroundCommand extends ElevatorCommand {
		ElevatorGroundCommand() {
			this.setTarget(Constants.ELEVATOR_TARGET_GROUND);
		}

		// @Override
		// public void execute() {
		// 	if (isBottomLimitReached()) {
		// 		elevatorMotor.set(0);
		// 		elevatorMotor.setPosition(Constants.ELEVATOR_PID_TARGET_GROUND);
		// 	} else {
		// 		elevatorMotor.setControl(
		// 			mmVoltage.withPosition(Constants.ELEVATOR_PID_TARGET_GROUND)
		// 		);
		// 	}
		// }
	}

	/** A command that moves the elevator to the Station position. */
	class ElevatorStationCommand extends ElevatorCommand {
		ElevatorStationCommand() {
			this.setTarget(Constants.ELEVATOR_TARGET_L2);
		}

		// @Override
		// public void execute() {
		// 	elevatorMotor.setControl(mmVoltage.withPosition(Constants.ELEVATOR_PID_TARGET_STATION));
		// }
	}

	/** A command that moves the elevator to the L4 position. */
	class ElevatorL4Command extends ElevatorCommand {
		ElevatorL4Command() {
			this.setTarget(Constants.ELEVATOR_TARGET_L4);
		}

		// @Override
		// public void execute() {
		// 	if (isTopLimitReached()) {
		// 		elevatorMotor.set(0);
		// 	} else {
		// 		elevatorMotor.setControl(
		// 			mmVoltage.withPosition(Constants.ELEVATOR_PID_TARGET_L4)
		// 		);
		// 	}
		// }
	}

	// FOR COMMANDS: JUST SET THE STATE (UPDATE IS STILL CALLED). INVESTIGATE WEEK 4.

	/**
	 * Creates a Command to move the elevator to the ground position.
	 * @return A new elevator ground command.
	 */
	public Command elevatorGroundCommand() {
		return new ElevatorGroundCommand();
	}

	/**
	 * Creates a Command to move the elevator to the station position.
	 * @return A new elevator station command.
	 */
	public Command elevatorStationCommand() {
		return new ElevatorStationCommand();
	}

	/**
	 * Creates a Command to move the elevator to the L4 position.
	 * @return A new elevator L4 command.
	 */
	public Command elevatorL4Command() {
		return new ElevatorL4Command();
	}
}
