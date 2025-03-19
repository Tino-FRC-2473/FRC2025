package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
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
	private MotionMagicVoltage motionRequest;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private TalonFX elevatorMotor;
	private DigitalInput groundLimitSwitch;

	private FunnelFSMSystem funnelSystem;

	/* ======================== Constructor ======================== */

	/**
	 * Create ElevatorFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 * @param funnelFSMSystem the funnel FSM.
	 */
	public ElevatorFSMSystem(FunnelFSMSystem funnelFSMSystem) {
		// Perform hardware init
		elevatorMotor = new TalonFXWrapper(HardwareMap.CAN_ID_ELEVATOR);

		motionRequest = new MotionMagicVoltage(0);

		var talonFXConfigs = new TalonFXConfiguration();

		var outputConfigs = talonFXConfigs.MotorOutput;
		outputConfigs.NeutralMode = NeutralModeValue.Brake;

		// apply sw limit
		var swLimitSwitch = talonFXConfigs.SoftwareLimitSwitch;
		swLimitSwitch.ForwardSoftLimitEnable = true; // enable top limit
		swLimitSwitch.ReverseSoftLimitEnable = true; // enable bottom limit
		swLimitSwitch.ForwardSoftLimitThreshold = Constants.ELEVATOR_UPPER_THRESHOLD
			.in(Units.Inches);
		swLimitSwitch.ReverseSoftLimitThreshold = Units.Inches.of(0).in(Units.Inches);

		var sensorConfig = talonFXConfigs.Feedback;
		sensorConfig.SensorToMechanismRatio = Constants.ELEVATOR_ROTS_TO_INCHES;

		var slot0Configs = talonFXConfigs.Slot0;
		slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
		slot0Configs.kG = Constants.ELEVATOR_KG;
		slot0Configs.kS = Constants.ELEVATOR_KS;
		slot0Configs.kV = Constants.ELEVATOR_KV;
		slot0Configs.kA = Constants.ELEVATOR_KA;
		slot0Configs.kP = Constants.ELEVATOR_KP;
		slot0Configs.kI = Constants.ELEVATOR_KI;
		slot0Configs.kD = Constants.ELEVATOR_KD;
		slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		var motionMagicConfigs = talonFXConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = Constants.ELEVATOR_CRUISE_VELO;
		motionMagicConfigs.MotionMagicAcceleration = Constants.ELEVATOR_TARGET_ACCEL;
		motionMagicConfigs.MotionMagicExpo_kV = Constants.ELEVATOR_EXPO_KV;

		elevatorMotor.getConfigurator().apply(talonFXConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
				Constants.UPDATE_FREQUENCY_HZ,
				elevatorMotor.getPosition(),
				elevatorMotor.getVelocity(),
				elevatorMotor.getAcceleration(),
				elevatorMotor.getMotorVoltage(),
				elevatorMotor.getRotorPosition(),
				elevatorMotor.getRotorVelocity()
		);

		elevatorMotor.optimizeBusUtilization();
			// MUST set brake after applying other configs

		// Initialize limit switch
		groundLimitSwitch = new DigitalInput(HardwareMap.ELEVATOR_GROUND_LIMIT_SWITCH_DIO_PORT);

		// Reset state machine

		elevatorMotor.setPosition(0);

		this.funnelSystem = funnelFSMSystem;

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

		if (!HardwareMap.useSuperStructure()) {
			handleOverrideState(input);
		}

		// telemetry and logging
		MechLogging.getInstance().updateElevatorPose3d(elevatorMotor.getPosition()
			.getValueAsDouble());

	}

	/**
	 * Updates the logging information for the elevator system.
	 */
	public void updateLogging() {
		Logger.recordOutput("Elevator encoder", elevatorMotor.getPosition().getValueAsDouble());
		Logger.recordOutput("Elevator velocity", elevatorMotor.getVelocity().getValueAsDouble());

		Logger.recordOutput("Elevator bottom limit switch pressed", isBottomLimitReached());

		Logger.recordOutput("Elevator State", currentState.toString());
		Logger.recordOutput("Elevator Voltage", elevatorMotor.getMotorVoltage().getValueAsDouble());

		Logger.recordOutput("Elevator Accel", elevatorMotor.getAcceleration().getValueAsDouble());

		Logger.recordOutput("Elev Inrage GRND?", inRange(getElevatorpos(),
			Constants.ELEVATOR_TARGET_GROUND));

		Logger.recordOutput("Elev Inrage L4?", inRange(getElevatorpos(),
			Constants.ELEVATOR_TARGET_L4));

		Logger.recordOutput("ROTR POS", elevatorMotor.getRotorPosition().getValueAsDouble());
		Logger.recordOutput("ROTR VELO", elevatorMotor.getRotorVelocity().getValueAsDouble());
	}

	/**
	 * Set the state of the ElevatorFSM.
	 * @param state The state to set the ElevatorFSM to.
	 */
	public void setState(ElevatorFSMState state) {
		currentState = state;
	}

	/**
	 * Handle the elevator states under manual superstructure control.
	 * @param input Global TeleopInput if robot in teleop mode or null if
 	 *        the robot is in autonomous mode.
	 */
	public void handleOverrideState(TeleopInput input) {
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
					&& !input.isL3ButtonPressed()
					&& !input.isArmLowStowButtonPressed()
					&& !input.isArmHighStowButtonPressed()) {
					return ElevatorFSMState.GROUND;
				}
				if (input.isL2ButtonPressed()
					&& funnelSystem.isHoldingCoral()
					&& !input.isL4ButtonPressed()
					&& !input.isGroundButtonPressed()
					&& !input.isL3ButtonPressed()
					&& !input.isArmLowStowButtonPressed()
					&& !input.isArmHighStowButtonPressed()) {
					return ElevatorFSMState.LEVEL2;
				}
				if (input.isL3ButtonPressed()
					&& funnelSystem.isHoldingCoral()
					&& !input.isL4ButtonPressed()
					&& !input.isGroundButtonPressed()
					&& !input.isL2ButtonPressed()
					&& !input.isArmLowStowButtonPressed()
					&& !input.isArmHighStowButtonPressed()) {
					return ElevatorFSMState.LEVEL3;
				}
				if (input.isL4ButtonPressed()
					&& funnelSystem.isHoldingCoral()
					&& !input.isGroundButtonPressed()
					&& !input.isL2ButtonPressed()
					&& !input.isL3ButtonPressed()
					&& !input.isArmLowStowButtonPressed()
					&& !input.isArmHighStowButtonPressed()) {
					return ElevatorFSMState.LEVEL4;
				}

			case GROUND:
				if (isBottomLimitReached() || inRange(getElevatorpos(),
					Constants.ELEVATOR_TARGET_GROUND)) {
					return ElevatorFSMState.MANUAL;
				}
				return ElevatorFSMState.GROUND;

			case LEVEL2:
				if (inRange(getElevatorpos(), Constants.ELEVATOR_TARGET_L2)) {
					return ElevatorFSMState.MANUAL;
				}
				return ElevatorFSMState.LEVEL2;

			case LEVEL3:
				if (inRange(getElevatorpos(), Constants.ELEVATOR_TARGET_L3)) {
					return ElevatorFSMState.MANUAL;
				}
				return ElevatorFSMState.LEVEL3;

			case LEVEL4:
				if (inRange(getElevatorpos(), Constants.ELEVATOR_TARGET_L4)) {
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
			return elevatorMotor.getPosition().getValueAsDouble()
				< Constants.ELEVATOR_INRANGE_VALUE.in(Units.Inches);
		}
		return groundLimitSwitch.get(); // switch is normally open
	}

	private boolean inRange(Distance currentPos, Distance targetPos) {
		return (currentPos.compareTo(targetPos.minus(Constants.ELEVATOR_INRANGE_VALUE)) > 0)
				&& currentPos.compareTo(targetPos.plus(Constants.ELEVATOR_INRANGE_VALUE)) < 0;
	}

	private Distance getElevatorpos() {
		return Units.Inches.of(elevatorMotor.getPosition().getValueAsDouble());
	}

	/* ------------------------ FSM state handlers ------------------------ */

	/**
	 * Handle behavior in MANUAL.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleManualState(TeleopInput input) {
		double signalInput = input.getManualElevatorMovementInput();
		signalInput = MathUtil.applyDeadband(signalInput,
			Constants.ELEVATOR_JOYSTICK_INPUT_DEADBAND);
		if (isBottomLimitReached()) {
			elevatorMotor.setPosition(0);
			if (signalInput < 0) {
				elevatorMotor.set(0); //don't go even further down if you hit the lower limit!
				return;
			}
		}

		if (signalInput == 0 && elevatorMotor.getPosition().getValueAsDouble()
			> Constants.KG_CHECK.in(Units.Inches)) {
			if (!Robot.isSimulation()) {
				elevatorMotor.setControl(new VoltageOut(Constants.ELEVATOR_KG));
			}
		} else {
			elevatorMotor.set(signalInput * Constants.ELEVATOR_MANUAL_SCALE);
		}
	}

	/**
	 * Handle behavior in GROUND.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleGroundState(TeleopInput input) {
		if (isBottomLimitReached()) {
			elevatorMotor.setPosition(0);
		} else {
			elevatorMotor.setControl(
				motionRequest.withPosition(Constants.ELEVATOR_TARGET_GROUND.in(Units.Inches))
			);
		}
	}

	/**
	 * Handle behavior in LEVEL2.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleL2State(TeleopInput input) {
		elevatorMotor.setControl(
			motionRequest.withPosition(Constants.ELEVATOR_TARGET_L2.in(Units.Inches))
		);
	}

	/**
	 * Handle behavior in LEVEL3.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleL3State(TeleopInput input) {
		elevatorMotor.setControl(
				motionRequest.withPosition(Constants.ELEVATOR_TARGET_L3.in(Units.Inches))
		);
	}

	/**
	 * Handle behavior in LEVEL4.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleL4State(TeleopInput input) {
		elevatorMotor.setControl(
				motionRequest.withPosition(Constants.ELEVATOR_TARGET_L4.in(Units.Inches))
		);
	}

	/**
	 * Is elevator at L2 boolean accessor.
	 * @return whether or not elevator is at L2.
	 */
	public boolean isElevatorAtL2() {
		return inRange(getElevatorpos(), Constants.ELEVATOR_TARGET_L2);
	}

	/**
	 * Is elevator at L3 boolean accessor.
	 * @return whether or not elevator is at L3.
	 */
	public boolean isElevatorAtL3() {
		return inRange(getElevatorpos(), Constants.ELEVATOR_TARGET_L3);
	}

	/**
	 * Is elevator at L4 boolean accessor.
	 * @return whether or not elevator is at L4.
	 */
	public boolean isElevatorAtL4() {
		return inRange(getElevatorpos(), Constants.ELEVATOR_TARGET_L4);
	}

	/**
	 * Is elevator at ground boolean accessor.
	 * @return whether or not elevator is at ground.
	 */
	public boolean isElevatorAtGround() {
		return isBottomLimitReached();
	}

	/* ---- Elevator Commands ---- */

	/** Superclass for elevator commands. */
	abstract class ElevatorCommand extends Command {
		private Distance target;

		@Override
		public void execute() {
			elevatorMotor.setControl(
				motionRequest.withPosition(target.in(Units.Inches))
			);

			if (Robot.isSimulation()) {
				MechLogging.getInstance().updateElevatorPose3d(
						elevatorMotor.getPosition().getValueAsDouble());
			}
		}

		@Override
		public boolean isFinished() {
			return inRange(getElevatorpos(), target);
		}

		@Override
		public void end(boolean interrupted) { }

		protected void setTarget(Distance newTarget) {
			this.target = newTarget;
		}

		protected Distance getTarget() {
			return this.target;
		}
	}

	/** A command that waits for one second. */
	class WaitCommand extends Command {
		private Timer timer;

		@Override
		public void initialize() {
			timer = new Timer();
			timer.reset();
			timer.start();
		}

		@Override
		public boolean isFinished() {
			return timer.get() > 1;
		}

		@Override
		public void end(boolean interrupted) {
			timer.reset();
		}
	}

	/** A command that moves the elevator to the Ground position. */
	class ElevatorGroundCommand extends ElevatorCommand {
		ElevatorGroundCommand() {
			this.setTarget(Constants.ELEVATOR_TARGET_GROUND);
		}
	}

	/** A command that moves the elevator to the L2 position. */
	class ElevatorL2Command extends ElevatorCommand {
		ElevatorL2Command() {
			this.setTarget(Constants.ELEVATOR_TARGET_L2);
		}
	}

	/** A command that moves the elevator to the L3 position. */
	class ElevatorL3Command extends ElevatorCommand {
		ElevatorL3Command() {
			this.setTarget(Constants.ELEVATOR_TARGET_L3);
		}
	}

	/** A command that moves the elevator to the L4 position. */
	class ElevatorL4Command extends ElevatorCommand {
		ElevatorL4Command() {
			this.setTarget(Constants.ELEVATOR_TARGET_L4);
		}
	}

	/**
	 * Creates a Command to move the elevator to the ground position.
	 * @return A new elevator ground command.
	 */
	public Command elevatorGroundCommand() {
		return new ElevatorGroundCommand();
	}

	/**
	 * Creates a Command to move the elevator to the L2 position.
	 * @return A new elevator L2 command.
	 */
	public Command elevatorL2Command() {
		return new ElevatorL2Command();
	}

	/**
	 * Creates a Command to move the elevator to the L3 position.
	 * @return A new elevator L3 command.
	 */
	public Command elevatorL3Command() {
		return new ElevatorL3Command();
	}

	/**
	 * Creates a Command to move the elevator to the L4 position.
	 * @return A new elevator L4 command.
	 */
	public Command elevatorL4Command() {
		return new ElevatorL4Command();
	}

	/**
	 * Creates a Command that waits for a specified duration.
	 * @return A new wait command.
	 */
	public Command waitCommand() {
		return new WaitCommand();
	}
}
