package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.units.measure.*;

import frc.robot.CommandSwerveDrivetrain;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class TunerConstants {
	// Both sets of gains need to be tuned to your individual robot.

	// The steer motor uses any SwerveModule.SteerRequestType control request with the
	// output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
	private static final Slot0Configs STEER_GAINS = new Slot0Configs()
		.withKP(DriveConstants.STEER_P).withKI(0).withKD(DriveConstants.STEER_D) // TODO: TUNE THE D
		.withKS(DriveConstants.STEER_S).withKV(DriveConstants.STEER_V).withKA(0)
		.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
	// When using closed-loop control, the drive motor uses the control
	// output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
	private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
		.withKP(DriveConstants.DRIVE_P).withKI(0).withKD(0)
		.withKS(0).withKV(DriveConstants.DRIVE_V);

	// The closed-loop output type to use for the steer motors;
	// This affects the PID/FF gains for the steer motors
	private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUT = ClosedLoopOutputType.Voltage;
	// The closed-loop output type to use for the drive motors;
	// This affects the PID/FF gains for the drive motors
	private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUT = ClosedLoopOutputType.Voltage;

	// The type of motor used for the drive motor
	private static final DriveMotorArrangement DRIVE_MOTOR_TYPE =
		DriveMotorArrangement.TalonFX_Integrated;
	// The type of motor used for the drive motor
	private static final SteerMotorArrangement STEER_MOTOR_TYPE =
		SteerMotorArrangement.TalonFX_Integrated;

	// The remote sensor feedback type to use for the steer motors;
	// When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
	private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

	// The stator current at which the wheels start to slip;
	// This needs to be tuned to your individual robot
	private static final Current SLIP_CURRENT_AMPS = Amps.of(120.0);

	// Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
	// Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
	private static final TalonFXConfiguration DRIVE_INIT_CONFIGS = new TalonFXConfiguration();
	private static final TalonFXConfiguration STEER_INIT_CONFIGS = new TalonFXConfiguration()
		.withCurrentLimits(
			new CurrentLimitsConfigs()
				// Swerve azimuth does not require much torque output,
				//so we can set a relatively low
				//stator current limit to help avoid brownouts without impacting performance.
				.withStatorCurrentLimit(Amps.of(60))
				.withStatorCurrentLimitEnable(true)
		);
	private static final CANcoderConfiguration ENCODER_INIT_CONGIGS = new CANcoderConfiguration();
	// Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
	private static final Pigeon2Configuration PIGEON_CONFIGS = null;

	// CAN bus that the devices are located on;
	// All swerve devices must share the same CAN bus
	public static final CANBus CANBUS = new CANBus("Drivetrain", "./logs/example.hoot");

	// Theoretical free speed (m/s) at 12 V applied output;
	// This needs to be tuned to your individual robot
	public static final LinearVelocity SPEED_AT_12_VOLTS = MetersPerSecond.of(2);

	// Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
	// This may need to be tuned to your individual robot
	private static final double COUPLE_RATIO = 0;

	private static final double DRIVE_GEAR_RATIO = 5.902777777777778;
	private static final double STEER_GEAR_RATIO = 15.42857142857143;
	private static final Distance WHEEL_RADIUS = Inches.of(2);

	private static final boolean INVERT_LEFT = false;
	private static final boolean INVERT_RIGHT = true;

	private static final int PIGEON_ID = 13;

	// These are only used for simulation
	private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
	private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
	// Simulated voltage necessary to overcome friction
	private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
	private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

	public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS =
		new SwerveDrivetrainConstants()
			.withCANBusName(CANBUS.getName())
			.withPigeon2Id(PIGEON_ID)
			.withPigeon2Configs(PIGEON_CONFIGS);

	private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration,
		CANcoderConfiguration> CONSTANT_CREATOR =
		new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration,
			CANcoderConfiguration>()
			.withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
			.withSteerMotorGearRatio(STEER_GEAR_RATIO)
			.withCouplingGearRatio(COUPLE_RATIO)
			.withWheelRadius(WHEEL_RADIUS)
			.withSteerMotorGains(STEER_GAINS)
			.withDriveMotorGains(DRIVE_GAINS)
			.withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
			.withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUT)
			.withSlipCurrent(SLIP_CURRENT_AMPS)
			.withSpeedAt12Volts(SPEED_AT_12_VOLTS)
			.withDriveMotorType(DRIVE_MOTOR_TYPE)
			.withSteerMotorType(STEER_MOTOR_TYPE)
			.withFeedbackSource(STEER_FEEDBACK_TYPE)
			.withDriveMotorInitialConfigs(DRIVE_INIT_CONFIGS)
			.withSteerMotorInitialConfigs(STEER_INIT_CONFIGS)
			.withEncoderInitialConfigs(ENCODER_INIT_CONGIGS)
			.withSteerInertia(STEER_INERTIA)
			.withDriveInertia(DRIVE_INERTIA)
			.withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
			.withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);


	// Front Left
	private static final int kFrontLeftDriveMotorId = 1;
	private static final int kFrontLeftSteerMotorId = 2;
	private static final int kFrontLeftEncoderId = 9;
	private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.324462890625);
	private static final boolean kFrontLeftSteerMotorInverted = true;
	private static final boolean kFrontLeftEncoderInverted = false;

	private static final Distance kFrontLeftXPos = Inches.of(14);
	private static final Distance kFrontLeftYPos = Inches.of(14);

	// Front Right
	private static final int kFrontRightDriveMotorId = 3;
	private static final int kFrontRightSteerMotorId = 4;
	private static final int kFrontRightEncoderId = 10;
	private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.398681640625);
	private static final boolean kFrontRightSteerMotorInverted = true;
	private static final boolean kFrontRightEncoderInverted = false;

	private static final Distance kFrontRightXPos = Inches.of(14);
	private static final Distance kFrontRightYPos = Inches.of(-14);

	// Back Left
	private static final int kBackLeftDriveMotorId = 7;
	private static final int kBackLeftSteerMotorId = 8;
	private static final int kBackLeftEncoderId = 12;
	private static final Angle kBackLeftEncoderOffset = Rotations.of(0.30078125);
	private static final boolean kBackLeftSteerMotorInverted = true;
	private static final boolean kBackLeftEncoderInverted = false;

	private static final Distance kBackLeftXPos = Inches.of(-14);
	private static final Distance kBackLeftYPos = Inches.of(14);

	// Back Right
	private static final int kBackRightDriveMotorId = 5;
	private static final int kBackRightSteerMotorId = 6;
	private static final int kBackRightEncoderId = 11;
	private static final Angle kBackRightEncoderOffset = Rotations.of(0.02978515625);
	private static final boolean kBackRightSteerMotorInverted = true;
	private static final boolean kBackRightEncoderInverted = false;

	private static final Distance kBackRightXPos = Inches.of(-14);
	private static final Distance kBackRightYPos = Inches.of(-14);


	public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration,
		CANcoderConfiguration> FRONT_LEFT =
		CONSTANT_CREATOR.createModuleConstants(
			kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId,
			kFrontLeftEncoderOffset, kFrontLeftXPos, kFrontLeftYPos, INVERT_LEFT,
			kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
		);
	public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
		CONSTANT_CREATOR.createModuleConstants(
			kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
			kFrontRightXPos, kFrontRightYPos, INVERT_RIGHT, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
		);
	public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
		CONSTANT_CREATOR.createModuleConstants(
			kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
			kBackLeftXPos, kBackLeftYPos, INVERT_LEFT, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
		);
	public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
		CONSTANT_CREATOR.createModuleConstants(
			kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
			kBackRightXPos, kBackRightYPos, INVERT_RIGHT, kBackRightSteerMotorInverted, kBackRightEncoderInverted
		);

	/**
	 * Creates a CommandSwerveDrivetrain instance.
	 * This should only be called once in your robot program.
	 * @return a new CommandSwerveDrivetrain instance
	 */
	public static CommandSwerveDrivetrain createDrivetrain() {
		return new CommandSwerveDrivetrain(
			DRIVETRAIN_CONSTANTS, FRONT_LEFT, FrontRight, BackLeft, BackRight
		);
	}
}
