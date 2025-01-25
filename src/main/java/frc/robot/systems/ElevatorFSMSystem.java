package frc.robot.systems;


// WPILib Imports

// Third party Hardware Imports
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
import frc.robot.motors.TalonFXWrapper;


public class ElevatorFSMSystem {

	/* ======================== Constants ======================== */

	// FSM state definitions
	public enum ElevatorFSMState {
		MANUAL,
		GROUND,
		STATION,
		LEVEL4
	}

	// Elevator comnad
	class MoveElevatorCommand extends Command {

		private double targetPos;

		MoveElevatorCommand(double target) {
			this.targetPos = target;
			elevatorTargetReached = false;
		}

		@Override
		public void execute() {
			handlePIDState(targetPos);
		}

		@Override
		public boolean isFinished() {
			return elevatorTargetReached;
		}

		@Override
		public void end(boolean interrupted) { }
	}

	// Target reched check for command
	private boolean elevatorTargetReached;

	/* ======================== Private variables ======================== */

	private ElevatorFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private TalonFX elevatorMotor;
	private final MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0);
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

		// elevatorMotor.setPosition(0); // reset kraken encoder (only use when tuning)
		elevatorMotor.setNeutralMode(NeutralModeValue.Brake);

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

		// Apply Motion Magic settings
		var motionMagicConfigs = talonFXConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = Constants.ELEVATOR_CONFIG_CONSTANT_CV;
		motionMagicConfigs.MotionMagicAcceleration = Constants.ELEVATOR_CONFIG_CONSTANT_A;
		motionMagicConfigs.MotionMagicJerk = Constants.ELEVATOR_CONFIG_CONSTANT_J;

		elevatorMotor.getConfigurator().apply(talonFXConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
				Constants.UPDATE_FREQUENCY_HZ,
				elevatorMotor.getPosition(),
				elevatorMotor.getVelocity(),
				elevatorMotor.getAcceleration(),
				elevatorMotor.getMotorVoltage()
		);

		elevatorMotor.optimizeBusUtilization();

		// Initialize limit switches
		groundLimitSwitch = new DigitalInput(HardwareMap.ELEVATOR_GROUND_LIMIT_SWITCH_PORT);
			//okay for stopping and resetting
		topLimitSwitch = new DigitalInput(HardwareMap.ELEVATOR_TOP_LIMIT_SWITCH_PORT);
			//only use to stop, DO NOT USE TO RESET

		// Reset state machine
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

		elevatorMotor.setPosition(Constants.ELEVATOR_PID_TARGET_GROUND);

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
			case STATION:
				handleStationState(input);
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
					&& !input.isL4ButtonPressed()
					&& !input.isStationButtonPressed()) {
					return ElevatorFSMState.GROUND;
				}
				if (input.isL4ButtonPressed()
					&& !input.isGroundButtonPressed()
					&& !input.isStationButtonPressed()) {
					return ElevatorFSMState.LEVEL4;
				}
				if (input.isStationButtonPressed()
					&& !input.isL4ButtonPressed()
					&& !input.isGroundButtonPressed()) {
					return ElevatorFSMState.STATION;
				}
				return ElevatorFSMState.MANUAL;

			case GROUND:
				if (!input.isGroundButtonPressed()) {
					return ElevatorFSMState.MANUAL;
				}
				return ElevatorFSMState.GROUND;

			case STATION:
				if (!input.isStationButtonPressed()) {
					return ElevatorFSMState.MANUAL;
				}
				return ElevatorFSMState.STATION;

			case LEVEL4:
				if (!input.isL4ButtonPressed()) {
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
		// return topLimitSwitch.get(); // switch is normally open
		return false;
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
		// SmartDashboard.putNumber("input", signalInput);
		if (isBottomLimitReached()) {
			elevatorMotor.setPosition(Constants.ELEVATOR_PID_TARGET_GROUND);
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

		elevatorMotor.set(Constants.ELEVATOR_MANUAL_SCALE * signalInput);
	}

	/**
	 * Handle behavior in GROUND.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleGroundState(TeleopInput input) {
		handlePIDState(Constants.ELEVATOR_PID_TARGET_GROUND);
	}

	/**
	 * Handle behavior in STATION.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleStationState(TeleopInput input) {
		handlePIDState(Constants.ELEVATOR_PID_TARGET_STATION);
	}

	/**
	 * Handle behavior in L4.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleL4State(TeleopInput input) {
		handlePIDState(Constants.ELEVATOR_PID_TARGET_L4);
	}

	/**
	 * Handle behavior for a generic auto state.
	 * @param target the target pid value to move to
	 */
	private void handlePIDState(double target) {
		if (isBottomLimitReached()) {
			elevatorMotor.set(0);
			elevatorMotor.setPosition(0);
		} else if (isTopLimitReached()) {
			elevatorMotor.set(0);
		} else {
			elevatorMotor.setControl(mmVoltage.withPosition(target));
			if (elevatorMotor.getVelocity().getValueAsDouble() == 0) {
				elevatorTargetReached = true;
			}
		}
	}

	/**
	 * Command to move the elevator to a target position.
	 * @param target the target pid value to move to
	 * @return move elevator command
	 */
	public Command moveElevatorCommand(double target) {
		return new MoveElevatorCommand(target);
	}
}
