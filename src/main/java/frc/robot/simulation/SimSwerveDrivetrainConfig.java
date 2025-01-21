package frc.robot.simulation;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// CTRE Imports
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

// WPI Imports
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

// Measures
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

// Units
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

// Local Imports
import frc.robot.constants.SimConstants;

/**
 * Configuration helper for the MapleSimSwerveDrivetrain class.
 */
public class SimSwerveDrivetrainConfig {
	private Mass robotMass;
	private Distance robotBumperWidth;
	private Distance robotBumperLength;
	private DCMotor moduleDriveMotor;
	private DCMotor moduleSteerMotor;
	private double wheelCOF;
	private Translation2d[] moduleLocations;
	private Pigeon2 pigeon = new Pigeon2(0);
	private SwerveModule<TalonFX, TalonFX, CANcoder>[] modules;
	private SwerveModuleConstants<
		TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] moduleConstants;

	/**
	 * Get the default configuration of a MapleSimSwerveDrivetrain.
	 * @return the default configuration
	 */
	public static SimSwerveDrivetrainConfig getDefault() {
		return new SimSwerveDrivetrainConfig()
			.withRobotMass(Pounds.of(SimConstants.MASS_WITH_BUMPER_LBS))
			.withBumperWidth(Inches.of(SimConstants.LENGTH_IN))
			.withBumperLength(Inches.of(SimConstants.WIDTH_IN))
			.withWheelCOF(SimConstants.WHEEL_COF)
			.withModuleDriveMotor(DCMotor.getKrakenX60(1))
			.withModuleSteerMotor(DCMotor.getKrakenX60(1));
	}

	/**
	 * Apply the mass of the robot <strong>with bumpers.</strong>
	 * @param robotMassWithBumper robot mass with bumpers
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withRobotMass(Mass robotMassWithBumper) {
		robotMass = robotMassWithBumper;
		return this;
	}

	/**
	 * Apply the length of the robot with the bumper on.
	 * @param bumperLength length of the robot with bumper
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withBumperLength(Distance bumperLength) {
		robotBumperLength = bumperLength;
		return this;
	}

	/**
	 * Apply the length of the robot with the bumper on.
	 * @param bumperWidth width of the robot with bumper
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withBumperWidth(Distance bumperWidth) {
		robotBumperWidth = bumperWidth;
		return this;
	}

	/**
	 * Applies the drive motor used for each of the modules in the drivetrain.
	 * @param modulesDriveMotor the drive motor used in the module
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withModuleDriveMotor(DCMotor modulesDriveMotor) {
		moduleDriveMotor = modulesDriveMotor;
		return this;
	}

	/**
	 * Applies the steer motor used for each of the modules in the drivetrain.
	 * @param modulesSteerMotor the steer motor used in the module
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withModuleSteerMotor(DCMotor modulesSteerMotor) {
		moduleSteerMotor = modulesSteerMotor;
		return this;
	}

	/**
	 * Applies the Coefficient of Friction of the wheel material used on the modules.
	 * @param wheelCoefficientOfFriction Coefficient of Friction of the wheel
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withWheelCOF(double wheelCoefficientOfFriction) {
		wheelCOF = wheelCoefficientOfFriction;
		return this;
	}

	/**
	 * Applies the module locations of the drivetrain.
	 * @param moduleLocs the locations of the modules
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withModuleLocations(Translation2d[] moduleLocs) {
		moduleLocations = moduleLocs;
		return this;
	}

	/**
	 * Applies the pigeon IMU that is being used in this drivetrain.
	 * @param pigeonIMU the pigeon IMU
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withPigeon(Pigeon2 pigeonIMU) {
		pigeon = pigeonIMU;
		return this;
	}

	/**
	 * Applies all the modules to this drivetrain config.
	 * @param swerveModules the modules used on this drivetrain
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withModules(
		SwerveModule<TalonFX, TalonFX, CANcoder>[] swerveModules) {
		modules = swerveModules;
		return this;
	}
	/**
	 * Applies the module constants for each of the modules in the drivetrain.
	 * @param moduleConsts the module constants
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withModuleConstants(
		SwerveModuleConstants<
			TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... moduleConsts
	) {
		moduleConstants = moduleConsts;
		return this;
	}

	/**
	 * Get the robot mass with bumpers.
	 * @return the robot mass with bumpers
	 */
	public Mass getRobotMass() {
		return robotMass;
	}

	/**
	 * Get the width of the robot with bumper.
	 * @return the with of the robot with the bumper on.
	 */
	public Distance getBumperWidth() {
		return robotBumperWidth;
	}

	/**
	 * Get the length of the robot with the bumper.
	 * @return the length of the robot with the bumper on.
	 */
	public Distance getBumperLength() {
		return robotBumperLength;
	}

	/**
	 * Get the drive motor used in the module.
	 * @return the drive motor type of the module
	 */
	public DCMotor getModuleDriveMotor() {
		return moduleDriveMotor;
	}

	/**
	 * Get the steer motor used in the module.
	 * @return the steer motor of the module
	 */
	public DCMotor getModuleSteerMotor() {
		return moduleSteerMotor;
	}

	/**
	 * Get the Coefficient of Friction of the module wheel.
	 * @return the COF of the wheel
	 */
	public double getWheelCOF() {
		return wheelCOF;
	}

	/**
	 * Get the locations of the modules.
	 * @return the locations of the modules.
	 */
	public Translation2d[] getModuleLocations() {
		return moduleLocations;
	}

	/**
	 * Get the pigeon IMU used in the drivetrain.
	 * @return the pigeon imu used in the module
	 */
	public Pigeon2 getPigeon() {
		return pigeon;
	}

	/**
	 * Get the swerve modules in the drivetrain.
	 * @return list of all the modules in the drivetrain
	 */
	public SwerveModule<TalonFX, TalonFX, CANcoder>[] getModules() {
		return modules;
	}

	/**
	 * Gets the module constants of all of the modules in the drivetrain.
	 * @return the module constants in the drivetrain
	 */
	public SwerveModuleConstants<
		TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] getModuleConstants() {
		return moduleConstants;
	}
}
