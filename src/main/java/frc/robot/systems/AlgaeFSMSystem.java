package frc.robot.systems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.HardwareMap;
import frc.robot.Robot;

// WPILib Imports

// Third party Hardware Imports

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.constants.Constants;
import frc.robot.motors.SparkMaxWrapper;

public class AlgaeFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum AlgaeFSMState {
		STOW_LOW,
		STOW_HIGH,
		REMOVE_ALGAE
	}

	/* ======================== Private variables ======================== */
	private AlgaeFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private SparkMax algaeMotor;
	private RelativeEncoder algaeMotorEncoder;
	private SparkClosedLoopController motionController;

	private final TrapezoidProfile armProfile = new TrapezoidProfile(
		new TrapezoidProfile.Constraints(
			Units.degreesToRadians(45),
			Units.degreesToRadians(90)));
	private TrapezoidProfile.State lastProfiledReference = new TrapezoidProfile.State();

	private final LinearSystem<N2, N1, N2> armSystem =
		LinearSystemId.createSingleJointedArmSystem(
			DCMotor.getNeo550(1),
			0.0105110621,
			15
		);

	private final KalmanFilter<N2, N1, N1> armKFilter =
		new KalmanFilter<>(
				Nat.N2(),
				Nat.N1(),
				(LinearSystem<N2, N1, N1>) armSystem.slice(0),
				VecBuilder.fill(0.015, 0.17),
				VecBuilder.fill(0.01),
				Constants.UPDATE_PERIOD_SECS);

	private final LinearQuadraticRegulator<N2, N1, N1> armLQRController =
		new LinearQuadraticRegulator<>(
			(LinearSystem<N2, N1, N1>) armSystem.slice(0),
			VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)),
			VecBuilder.fill(12.0),
			Constants.UPDATE_PERIOD_SECS);

	private final LinearSystemLoop<N2, N1, N1> armControlLoop =
		new LinearSystemLoop<>(
			(LinearSystem<N2, N1, N1>) armSystem.slice(0),
			armLQRController,
			armKFilter,
			12.0,
			Constants.UPDATE_PERIOD_SECS);

	/* ======================== Constructor ======================== */
	/**
	 * Create a AlgaeFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public AlgaeFSMSystem() {
		// Perform hardware init
		algaeMotor = new SparkMaxWrapper(HardwareMap.CAN_ID_ALGAE_REMOVER, MotorType.kBrushless);

		var sparkConfig = new SparkMaxConfig();
		var closedLoopConfig = sparkConfig.closedLoop;
		var maxMotioinConfigs = closedLoopConfig.maxMotion;
		var encoderConfig = sparkConfig.encoder;
		var softLimitConfig = sparkConfig.softLimit;
		var signalConfig = sparkConfig.signals;
		var rpsps = RadiansPerSecondPerSecond;

		maxMotioinConfigs
			.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0)
			.maxVelocity(Constants.ARM_MAX_VELO.in(RadiansPerSecond), ClosedLoopSlot.kSlot0)
			.maxAcceleration(Constants.ARM_MAX_ACCEL.in(rpsps), ClosedLoopSlot.kSlot0)
			.allowedClosedLoopError(Constants.ARM_MARGIN_ERR.in(Radians), ClosedLoopSlot.kSlot0);

		closedLoopConfig
			.p(0, ClosedLoopSlot.kSlot0)
			.i(0, ClosedLoopSlot.kSlot0)
			.d(0, ClosedLoopSlot.kSlot0)
			.iMaxAccum(0, ClosedLoopSlot.kSlot0)
			.outputRange(-1, 1, ClosedLoopSlot.kSlot0)
			.apply(maxMotioinConfigs);

		encoderConfig
				.positionConversionFactor(Constants.ARM_GEAR_RATIO)
				.velocityConversionFactor(Constants.ARM_GEAR_RATIO);

		softLimitConfig
			.forwardSoftLimitEnabled(true)
			.forwardSoftLimit(0)
			.reverseSoftLimitEnabled(true)
			.reverseSoftLimit(0);

		signalConfig
			.primaryEncoderPositionPeriodMs(Constants.UPDATE_PERIOD_MS)
			.primaryEncoderVelocityPeriodMs(Constants.UPDATE_PERIOD_MS)
			.busVoltagePeriodMs(Constants.UPDATE_PERIOD_MS)
			.appliedOutputPeriodMs(Constants.UPDATE_PERIOD_MS)
			.motorTemperaturePeriodMs(Constants.UPDATE_PERIOD_MS)
			.outputCurrentPeriodMs(Constants.UPDATE_PERIOD_MS);

		sparkConfig
			.idleMode(IdleMode.kBrake)
			.smartCurrentLimit(Constants.ARM_CURRENT_LIMIT)
			.inverted(false)
			.apply(closedLoopConfig)
			.apply(softLimitConfig)
			.apply(signalConfig)
			.apply(encoderConfig);

		algaeMotor.configure(
			sparkConfig,
			ResetMode.kNoResetSafeParameters,
			PersistMode.kPersistParameters);

		algaeMotorEncoder = algaeMotor.getEncoder();
		motionController = algaeMotor.getClosedLoopController();

		// Reset state machine
		algaeMotorEncoder.setPosition(Constants.ARM_START_POS.in(Radians));
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
		currentState = AlgaeFSMState.STOW_LOW;

		armControlLoop.reset(
			VecBuilder.fill(algaeMotorEncoder.getPosition(), algaeMotorEncoder.getVelocity()));

		lastProfiledReference =
			new TrapezoidProfile.State(
				algaeMotorEncoder.getPosition(),
				algaeMotorEncoder.getVelocity()
			);

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
			case STOW_LOW:
				handleStowLowState(input);
				break;

			case STOW_HIGH:
				handleStowHighState(input);
				break;

			case REMOVE_ALGAE:
				handleRemoveAlgaeState(input);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		if (Robot.isSimulation()) {
			nextState(input);
		}
	}

	/**
	 * Calls all logging and telemetry to be updated periodically.
	 */
	public void updateLogging() {
		Logger.recordOutput("Algae Arm/Position Radians", algaeMotorEncoder.getPosition());
		Logger.recordOutput("Algae Arm/Velocity DPS", algaeMotorEncoder.getVelocity());
		Logger.recordOutput("Algae Arm/Motor Voltage", algaeMotor.getBusVoltage());
		Logger.recordOutput("Algae Arm/Motor Current", algaeMotor.getOutputCurrent());
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
			case STOW_LOW:
				if (input.isArmHighStowButtonPressed()) {
					return AlgaeFSMState.STOW_HIGH;
				}

				if (input.isArmRemoveAlgaeButtonPressed()) {
					return AlgaeFSMState.REMOVE_ALGAE;
				}

				return AlgaeFSMState.STOW_LOW;

			case STOW_HIGH:
				if (input.isArmLowStowButtonPressed()) {
					return AlgaeFSMState.STOW_LOW;
				}

				if (input.isArmRemoveAlgaeButtonPressed()) {
					return AlgaeFSMState.REMOVE_ALGAE;
				}

				return AlgaeFSMState.STOW_HIGH;

			case REMOVE_ALGAE:
				if (input.isArmLowStowButtonPressed()) {
					return AlgaeFSMState.STOW_LOW;
				}

				if (input.isArmHighStowButtonPressed()) {
					return AlgaeFSMState.STOW_HIGH;
				}

				return AlgaeFSMState.REMOVE_ALGAE;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	private void runLQRControlLoop(Angle position) {
		TrapezoidProfile.State goalPos;
		goalPos = new TrapezoidProfile.State(position.in(Radians), 0.0);
		lastProfiledReference =
			armProfile.calculate(Constants.UPDATE_PERIOD_SECS, lastProfiledReference, goalPos);
		armControlLoop.setNextR(lastProfiledReference.position, lastProfiledReference.velocity);
		armControlLoop.correct(VecBuilder.fill(algaeMotorEncoder.getPosition()));
		armControlLoop.predict(Constants.UPDATE_PERIOD_SECS);
		double nextVoltage = armControlLoop.getU(0);
		algaeMotor.setVoltage(nextVoltage);
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in STOW.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleStowLowState(TeleopInput input) {
		motionController.setReference(
			Constants.ARM_TARGET_STOWED_LOW.in(Radians),
			ControlType.kMAXMotionPositionControl,
			ClosedLoopSlot.kSlot0);

		// runLQRControlLoop(Constants.ARM_TARGET_STOWED_LOW);
	}

	/**
	 * Handle behavior in DEPLOY.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleStowHighState(TeleopInput input) {
		motionController.setReference(
			Constants.ARM_TARGET_STOWED_HIGH.in(Radians),
			ControlType.kMAXMotionPositionControl,
			ClosedLoopSlot.kSlot0);

		// runLQRControlLoop(Constants.ARM_TARGET_STOWED_HIGH);
	}

	private void handleRemoveAlgaeState(TeleopInput input) {
		motionController.setReference(
			Constants.ARM_TARGET_ALGAE.in(Radians),
			ControlType.kMAXMotionPositionControl,
			ClosedLoopSlot.kSlot0);

		// runLQRControlLoop(Constants.ARM_TARGET_ALGAE);
	}
}
