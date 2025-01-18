package frc.robot.simulation;

// CTRE Imports
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

// WPI Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Feet;
// Units
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

// Maple-Sim
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

// Local Imports
import frc.robot.constants.SimConstants;
import frc.robot.Robot;

/*
 * This class has been taken from Maple-Sim
 */

/**
 * <h2>Injects Maple-Sim simulation data into a CTRE swerve drivetrain.</h2>
 *
 * <p>This class retrieves simulation data from Maple-Sim and injects it
 * into the CTRE {@link com.ctre.phoenix6.swerve.SwerveDrivetrain} instance.</p>
 */
public class MapleSimSwerveDrivetrain {
	private final Pigeon2SimState pigeonSim;
	private final SimSwerveModule[] simModules;
	private final SwerveDriveSimulation mapleSimDrive;

	/**
	 * Constructs a MapleSimSwerveDrivetrain with specified config.
	 * @param config the config to apply
	 */
	public MapleSimSwerveDrivetrain(SimSwerveDrivetrainConfig config) {
		pigeonSim = config.getPigeon().getSimState();
		simModules = new SimSwerveModule[config.getModules().length];

		var simulationConfig = DriveTrainSimulationConfig.Default()
				.withRobotMass(config.getRobotMass())
				.withBumperSize(config.getBumperWidth(), config.getBumperLength())
				.withGyro(COTS.ofPigeon2())
				.withCustomModuleTranslations(config.getModuleLocations())
				.withSwerveModule(new SwerveModuleSimulationConfig(
						config.getModuleDriveMotor(),
						config.getModuleSteerMotor(),
						config.getModuleConstants()[0].DriveMotorGearRatio,
						config.getModuleConstants()[0].SteerMotorGearRatio,
						Volts.of(config.getModuleConstants()[0].DriveFrictionVoltage),
						Volts.of(config.getModuleConstants()[0].SteerFrictionVoltage),
						Meters.of(config.getModuleConstants()[0].WheelRadius),
						KilogramSquareMeters.of(config.getModuleConstants()[0].SteerInertia),
						config.getWheelCOF()));

		mapleSimDrive = new SwerveDriveSimulation(
			simulationConfig,
			new Pose2d(
				new Translation2d(
					Feet.of(SimConstants.STARTING_POS_X_FT),
					Feet.of(SimConstants.STARTING_POS_Y_FT)
				),
				new Rotation2d()
			)
		);

		SwerveModuleSimulation[] moduleSimulations = mapleSimDrive.getModules();

		for (int i = 0; i < this.simModules.length; i++) {
			simModules[i] = new SimSwerveModule(
				config.getModuleConstants()[0], moduleSimulations[i], config.getModules()[i]);
		}

		SimulatedArena.overrideSimulationTimings(config.getSimPeriod(), 1);
		SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
	}

	/**
	 * Gets the SwerveModuleSimulation injected into the SwerveDrivetrain.
	 * @return the swerveModuleSimulation of this drivetrain
	 */
	public SwerveDriveSimulation getDriveSimulation() {
		return mapleSimDrive;
	}

	/**
	 * <h2>Update the simulation.</h2>
	 *
	 * <p>Updates the Maple-Sim simulation and injects the results into the simulated CTRE devices,
	 * including motors and the IMU.</p>
	 */
	public void update() {
		SimulatedArena.getInstance().simulationPeriodic();
		pigeonSim.setRawYaw(
				mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
		pigeonSim.setAngularVelocityZ(RadiansPerSecond.of(
				mapleSimDrive
					.getDriveTrainSimulatedChassisSpeedsRobotRelative().omegaRadiansPerSecond));
	}

	/**
	 * <h1>Represents the simulation of a single {@link SwerveModule}.</h1>
	 */
	protected static class SimSwerveModule {
		private final SwerveModuleConstants<
			TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> moduleConstant;
		private final SwerveModuleSimulation moduleSimulation;

		public SimSwerveModule(
				SwerveModuleConstants<
					TalonFXConfiguration,
					TalonFXConfiguration,
					CANcoderConfiguration> moduleConstants,
				SwerveModuleSimulation swerveModuleSimulation,
				SwerveModule<TalonFX, TalonFX, CANcoder> module) {
			this.moduleConstant = moduleConstants;
			this.moduleSimulation = swerveModuleSimulation;
			moduleSimulation.useDriveMotorController(
				new TalonFXMotorControllerSim(module.getDriveMotor()));
			moduleSimulation.useSteerMotorController(
					new TalonFXMotorControllerWithRemoteCanCoderSim(
						module.getSteerMotor(), module.getEncoder()));
		}

		public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration,
			CANcoderConfiguration> getModuleConstants() {
			return moduleConstant;
		}
	}

	/**
	 * <h2>Regulates all {@link SwerveModuleConstants} for a drivetrain simulation.</h2>
	 *
	 * <p>This method processes an array of {@link SwerveModuleConstants}
	 * to apply necessary adjustments
	 * for simulation purposes, ensuring compatibility and avoiding known bugs.</p>
	 * @param moduleConstants unregulated moduleConstants for each swerve module
	 * @return regulated swerve module constants as per sim requirements
	 * @see #regulateModuleConstantForSimulation(SwerveModuleConstants)
	 */
	public static SwerveModuleConstants<?, ?, ?>[]
		regulateModuleConstantsForSimulation(
			SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
		for (SwerveModuleConstants<?, ?, ?> moduleConstant : moduleConstants) {
			regulateModuleConstantForSimulation(moduleConstant);
		}

		return moduleConstants;
	}

	/**
	 * <h2>Regulates the {@link SwerveModuleConstants} for a single module.</h2>
	 *
	 * This method applies specific adjustments to the {@link SwerveModuleConstants} for simulation
	 * purposes. These changes have no effect on real robot operations and address
	 * known simulation bugs:
	 *
	 * <ul>
	 *     <li><strong>Inverted Drive Motors:</strong>
	 * 			Prevents drive PID issues caused by inverted configurations.</li>
	 *     <li><strong>Non-zero CanCoder Offsets:</strong>
	 * 			Fixes potential module state optimization issues.</li>
	 *     <li><strong>Steer Motor PID:</strong>
	 * 			Adjusts PID values tuned for real robots to improve simulation performance.</li>
	 * </ul>
	 *
	 * <h4>Note:This function is skipped when running on a real robot,
	 * 		ensuring no impact on constants used on real robot hardware.</h4>
	 * @param moduleConstants unregulated module constants that
	 * 					      are used on the real robot.
	 */
	private static void regulateModuleConstantForSimulation(
		SwerveModuleConstants<?, ?, ?> moduleConstants) {
		// Skip regulation if running on a real robot
		if (Robot.isReal()) {
			return;
		}

		// Apply simulation-specific adjustments to module constants
		moduleConstants
				// Disable encoder offsets
				.withEncoderOffset(0)
				// Disable motor inversions for drive and steer motors
				.withDriveMotorInverted(false)
				.withSteerMotorInverted(false)
				// Disable CanCoder inversion
				.withEncoderInverted(false)
				// Adjust steer motor PID gains for simulation
				.withSteerMotorGains(moduleConstants.SteerMotorGains
						.withKP(SimConstants.MODULE_STEER_P)  // Proportional gain
						.withKD(SimConstants.MODULE_STEER_D)) // Derivative gain
				// Adjust friction voltages
				.withDriveFrictionVoltage(Volts.of(SimConstants.DRIVE_FRICTION_VOLTS))
				.withSteerFrictionVoltage(Volts.of(SimConstants.STEER_FRICTION_VOLTS))
				// Adjust steer inertia
				.withSteerInertia(KilogramSquareMeters.of(SimConstants.STEER_INERTIA_KGMS2));
	}

	// Static Util Classes
	public static class TalonFXMotorControllerSim implements SimulatedMotorController {
		private final int id;
		private final TalonFXSimState talonFXSimState;

		/**
		 * Constructs a TalonFX MotorController which uses it's sim state.
		 * @param talonFX
		 */
		public TalonFXMotorControllerSim(TalonFX talonFX) {
			this.id = talonFX.getDeviceID();
			this.talonFXSimState = talonFX.getSimState();
		}

		@Override
		public Voltage updateControlSignal(
				Angle mechanismAngle,
				AngularVelocity mechanismVelocity,
				Angle encoderAngle,
				AngularVelocity encoderVelocity) {
			talonFXSimState.setRawRotorPosition(encoderAngle);
			talonFXSimState.setRotorVelocity(encoderVelocity);
			talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

			return talonFXSimState.getMotorVoltageMeasure();
		}

		/**
		 * Gets the Id of this talonfx motor controller.
		 * @return the id of the talonfx motor controller
		 */
		public int getId() {
			return id;
		}
	}

	public static class TalonFXMotorControllerWithRemoteCanCoderSim extends
		TalonFXMotorControllerSim {
		private final int encoderId;
		private final CANcoderSimState remoteCancoderSimState;

		/**
		 * Constructs a TalonFX motor controller but with a cancoder sim.
		 * @param talonFX motor to use
		 * @param cancoder cancoder used for sim state
		 */
		public TalonFXMotorControllerWithRemoteCanCoderSim(
			TalonFX talonFX,
			CANcoder cancoder
		) {
			super(talonFX);
			this.remoteCancoderSimState = cancoder.getSimState();

			this.encoderId = cancoder.getDeviceID();
		}

		@Override
		public Voltage updateControlSignal(
				Angle mechanismAngle,
				AngularVelocity mechanismVelocity,
				Angle encoderAngle,
				AngularVelocity encoderVelocity) {
			remoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
			remoteCancoderSimState.setRawPosition(mechanismAngle);
			remoteCancoderSimState.setVelocity(mechanismVelocity);

			return super.updateControlSignal(
				mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
		}
		/**
		 * Get the identifier of the encoder paired with this motor.
		 * @return the identifier of this sim motor
		 */
		public int getEncoderID() {
			return encoderId;
		}
	}
}
