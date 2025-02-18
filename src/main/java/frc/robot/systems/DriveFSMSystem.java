package frc.robot.systems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
// WPILib Imports
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
//CTRE Imports
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import choreo.auto.AutoFactory;

import com.ctre.phoenix6.swerve.SwerveRequest;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.SimConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.simulation.MapleSimSwerveDrivetrain;
import frc.robot.simulation.RaspberryPiSim;
import frc.robot.logging.SwerveLogging;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.RaspberryPi;
import frc.robot.AprilTag;

public class DriveFSMSystem extends SubsystemBase {
	/* ======================== Constants ======================== */

	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE,
		ALIGN_TO_REEF_TAG_STATE,
		ALIGN_TO_STATION_TAG_STATE
	}

	private static final double MAX_SPEED = TunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond);
		// kSpeedAt12Volts desired top speed
	private static final double MAX_ANGULAR_RATE =
		RotationsPerSecond.of(DriveConstants.MAX_ANGULAR_VELO_RPS).in(RadiansPerSecond);
		//3/4 rps angle velo

	private final SwerveRequest.FieldCentric drive
		= new SwerveRequest.FieldCentric()
		.withDeadband(MAX_SPEED * DriveConstants.DRIVE_DEADBAND) // 4% deadband
		.withRotationalDeadband(MAX_ANGULAR_RATE
		* DriveConstants.ROTATION_DEADBAND) //4% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop for drive motors
	private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle
		= new SwerveRequest.FieldCentricFacingAngle()
		.withDeadband(MAX_SPEED * DriveConstants.DRIVE_DEADBAND) // 4% deadband
		.withRotationalDeadband(MAX_ANGULAR_RATE * DriveConstants.ROTATION_DEADBAND) //4% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop for drive motors
	private final SwerveRequest.RobotCentric driveRobotCentric
		= new SwerveRequest.RobotCentric()
		.withDeadband(MAX_SPEED * DriveConstants.DRIVE_DEADBAND) // 4% deadband
		.withRotationalDeadband(MAX_ANGULAR_RATE * DriveConstants.ROTATION_DEADBAND) //4% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop for drive motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

	private final SwerveLogging logger = new SwerveLogging(MAX_SPEED);

	private CommandSwerveDrivetrain drivetrain;

	/* -- cv constants -- */
	private RaspberryPi rpi = new RaspberryPi();
	private Rotation2d rotationAlignmentPose;

	private double alignmentYOff = 0;

	private int[] blueReefTagArray = new int[] {
		AutoConstants.B_REEF_1_TAG_ID,
		AutoConstants.B_REEF_2_TAG_ID,
		AutoConstants.B_REEF_3_TAG_ID,
		AutoConstants.B_REEF_4_TAG_ID,
		AutoConstants.B_REEF_5_TAG_ID,
		AutoConstants.B_REEF_6_TAG_ID
	};
	private int[] redReefTagArray = new int[] {
		AutoConstants.R_REEF_1_TAG_ID,
		AutoConstants.R_REEF_2_TAG_ID,
		AutoConstants.R_REEF_3_TAG_ID,
		AutoConstants.R_REEF_4_TAG_ID,
		AutoConstants.R_REEF_5_TAG_ID,
		AutoConstants.R_REEF_6_TAG_ID
	};

	private int[] blueStationTagArray = new int[] {
		AutoConstants.BLUE_L_STATION_ID,
		AutoConstants.BLUE_R_STATION_ID
	};
	private int[] redStationTagArray = new int[] {
		AutoConstants.RED_L_STATION_ID,
		AutoConstants.RED_R_STATION_ID
	};

	private IntSupplier allianceOriented = () -> {
		if (!DriverStation.getAlliance().isPresent()) {
			return -1;
		}
		return DriverStation.getAlliance().get() == Alliance.Red ? 1 : -1;
	};

	private SlewRateLimiter slewRateX;
	private SlewRateLimiter slewRateY;
	private SlewRateLimiter slewRateA;

	private Comparator<AprilTag> aComparator = new Comparator<AprilTag>() {
		@Override
		public int compare(AprilTag o1, AprilTag o2) {
			return o1.compareTo(o2);
		}
	};

	private AprilTagFieldLayout aprilTagFieldLayout;
	private ArrayList<Pose2d> aprilTagRefPoses;

	/* ---- all drive to pose related constants ---- */
	private final ProfiledPIDController driveController =
		new ProfiledPIDController(
			0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
	private final ProfiledPIDController thetaController =
		new ProfiledPIDController(
			0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));

	private Translation2d lastSetpointTranslation = new Translation2d();
	private double driveErrorAbs = 0.0;
	private double thetaErrorAbs = 0.0;
	private boolean driveToPoseRunning = false;

	private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
	private DoubleSupplier omegaFF = () -> 0.0;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	/* ======================== Constructor ======================== */
	/**
	 * Create DriveFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		// Perform hardware init
		drivetrain = TunerConstants.createDrivetrain();
		rpi = (Utils.isSimulation()) ? new RaspberryPiSim() : new RaspberryPi();

		slewRateX = new SlewRateLimiter(DriveConstants.SLEW_RATE);
		slewRateY = new SlewRateLimiter(DriveConstants.SLEW_RATE);
		slewRateA = new SlewRateLimiter(DriveConstants.SLEW_RATE);

		driveController.setP(AutoConstants.ALIGN_DRIVE_P);
		driveController.setD(AutoConstants.ALIGN_DRIVE_D);
		driveController.setConstraints(
			new TrapezoidProfile.Constraints(
				AutoConstants.ALIGN_MAX_T_SPEED,
				AutoConstants.ALIGN_MAX_T_ACCEL
			));
		driveController.setTolerance(AutoConstants.DRIVE_TOLERANCE);
		thetaController.setP(AutoConstants.ALIGN_THETA_P);
		thetaController.setD(AutoConstants.ALIGN_THETA_D);
		thetaController.setConstraints(
			new TrapezoidProfile.Constraints(
				AutoConstants.ALIGN_MAX_R_SPEED,
				AutoConstants.ALIGN_MAX_R_ACCEL
			));
		thetaController.setTolerance(
			AutoConstants.THETA_TOLERANCE
		);

		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		try {
			aprilTagFieldLayout
				= new AprilTagFieldLayout(VisionConstants.APRIL_TAG_FIELD_LAYOUT_JSON);
		} catch (IOException e) {
			e.printStackTrace();
		}

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
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
		currentState = FSMState.TELEOP_STATE;
		rotationAlignmentPose = drivetrain.getState().Pose.getRotation();
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
		drivetrain.applyOperatorPerspective();

		if (input == null) {
			return;
		}

		updateVisionEstimates();

		switch (currentState) {
			case TELEOP_STATE:
				handleTeleOpState(input);
				break;
			case ALIGN_TO_REEF_TAG_STATE:
				handleReefTagAlignment(input);
				break;
			case ALIGN_TO_STATION_TAG_STATE:
				handleStationTagAlignment(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

	/**
	 * Performs specific action based on the autoState passed in.
	 */
	public void updateAutonomous() {
		logger.applyStateLogging(drivetrain.getState());
	}

	/**
	 * Follow the given trajectory sample.
	 * @return An AutoFactory instance for creating autonomous routines.
	 */
	public AutoFactory createAutoFactory() {
		return new AutoFactory(
			() -> drivetrain.getState().Pose,
			drivetrain::resetPose,
			drivetrain::followTrajectory,
			true,
			this
		);
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
	private FSMState nextState(TeleopInput input) {

		switch (currentState) {
			case TELEOP_STATE:
				if (input.getDriveSquareButton()) {
					return FSMState.ALIGN_TO_REEF_TAG_STATE;
				}  else if (input.getDriveTriangleButton()) {
					return FSMState.ALIGN_TO_STATION_TAG_STATE;
				} else {
					return FSMState.TELEOP_STATE;
				}
			case ALIGN_TO_REEF_TAG_STATE:
				if (input.getDriveSquareButton()) {
					return FSMState.ALIGN_TO_REEF_TAG_STATE;
				}  else if (input.getDriveTriangleButton()) {
					return FSMState.ALIGN_TO_STATION_TAG_STATE;
				} else {
					return FSMState.TELEOP_STATE;
				}
			case ALIGN_TO_STATION_TAG_STATE:
				if (input.getDriveSquareButton()) {
					return FSMState.ALIGN_TO_REEF_TAG_STATE;
				}  else if (input.getDriveTriangleButton()) {
					return FSMState.ALIGN_TO_STATION_TAG_STATE;
				} else {
					return FSMState.TELEOP_STATE;
				}

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleOpState(TeleopInput input) {
		logger.applyStateLogging(drivetrain.getState());

		double xSpeed = -MathUtil.applyDeadband(
			slewRateX.calculate(input.getDriveLeftJoystickY()), DriveConstants.JOYSTICK_DEADBAND
			) * MAX_SPEED / DriveConstants.SPEED_DAMP_FACTOR;
			// Drive forward with negative Y (forward) ^

		double ySpeed = -MathUtil.applyDeadband(
			slewRateY.calculate(input.getDriveLeftJoystickX()), DriveConstants.JOYSTICK_DEADBAND
			) * MAX_SPEED / DriveConstants.SPEED_DAMP_FACTOR;
			// Drive left with negative X (left) ^

		double rotXComp = -MathUtil.applyDeadband(
			slewRateA.calculate(input.getDriveRightJoystickX()), DriveConstants.JOYSTICK_DEADBAND
			) * MAX_ANGULAR_RATE;
			// Drive left with negative X (left) ^

		if (rotXComp != 0) {
			rotationAlignmentPose =
				(Utils.isSimulation())
					? getMapleSimDrivetrain().getDriveSimulation()
					.getSimulatedDriveTrainPose().getRotation()
					: drivetrain.getState().Pose.getRotation();
		}

		// System.out.println("TELEOP X:" + drivetrain.getState().Pose.getX());

		drivetrain.setControl(
			driveFacingAngle.withVelocityX(xSpeed * allianceOriented.getAsInt())
			.withVelocityY(ySpeed * allianceOriented.getAsInt())
			.withTargetDirection(rotationAlignmentPose)
			.withTargetRateFeedforward(rotXComp)
		);

		if (input.getDriveCircleButton()) {
			drivetrain.setControl(brake);
		}

		if (input.getDriveBackButtonPressed()) {
			drivetrain.seedFieldCentric();
		}
	}

	/**
	 * Update vision measurements according to all seen tags.
	 */
	public void updateVisionEstimates() {
		aprilTagRefPoses = new ArrayList<Pose2d>();
		ArrayList<AprilTag> reefTags = rpi.getReefAprilTags();
		ArrayList<AprilTag> stationTags = rpi.getStationAprilTags();
		reefTags.sort(aComparator);

		Pose2d currPose;

		if (Utils.isSimulation()) {
			currPose = getMapleSimDrivetrain().getDriveSimulation().getSimulatedDriveTrainPose();
		} else {
			currPose = drivetrain.getState().Pose;
		}

		for (int t = 0; t < reefTags.size(); t++) {
			AprilTag tag = reefTags.get(t);

			Optional<Pose3d> aprilTagPose3d = aprilTagFieldLayout.getTagPose(tag.getTagID());
			Pose2d alignmentPose2d = currPose
				.plus(new Transform2d(
					tag.getZ(),
					tag.getX(),
					new Rotation2d()));

			Transform2d robotToCamera =
				new Transform2d(
					SimConstants.ROBOT_TO_REEF_CAMERA.getTranslation().toTranslation2d(),
					SimConstants.ROBOT_TO_REEF_CAMERA.getRotation().toRotation2d()
				);

			if (!aprilTagPose3d.isEmpty()) {
				Pose2d imposedPose = new Pose2d(
					new Pose3d(currPose)
					.plus(aprilTagPose3d.get().minus(new Pose3d(alignmentPose2d)))
					.toPose2d().getTranslation(),
					currPose.getRotation()
				).transformBy(
					robotToCamera.inverse()
				);

				if (imposedPose.getTranslation().getDistance(currPose.getTranslation())
					< VisionConstants.LOCALIZATION_TRANSLATIONAL_THRESHOLD
					&& (reefTags.size() + stationTags.size())
						>= VisionConstants.LOCALIZATION_TAG_NUM) {

					aprilTagRefPoses.add(
						imposedPose
					);

					drivetrain.addVisionMeasurement(imposedPose, Timer.getFPGATimestamp());
				}
			}
		}

		for (int t = 0; t < stationTags.size(); t++) {
			AprilTag tag = stationTags.get(t);

			Optional<Pose3d> aprilTagPose3d = aprilTagFieldLayout.getTagPose(tag.getTagID());
			Pose2d alignmentPose2d = currPose
				.plus(new Transform2d(
					tag.getZ(),
					tag.getX(),
					new Rotation2d()));

			Transform2d robotToCamera =
				new Transform2d(
					SimConstants.ROBOT_TO_STATION_CAMERA.getTranslation().toTranslation2d(),
					SimConstants.ROBOT_TO_STATION_CAMERA.getRotation().toRotation2d()
				);

			if (!aprilTagPose3d.isEmpty()) {
				Pose2d imposedPose = new Pose2d(
					new Pose3d(currPose)
					.plus(aprilTagPose3d.get().minus(new Pose3d(alignmentPose2d)))
					.toPose2d().getTranslation(),
					currPose.getRotation()
				).transformBy(
					robotToCamera.inverse()
				);

				if (imposedPose.getTranslation().getDistance(currPose.getTranslation())
					< VisionConstants.LOCALIZATION_TRANSLATIONAL_THRESHOLD
					&& (reefTags.size() + stationTags.size())
						>= VisionConstants.LOCALIZATION_TAG_NUM) {

					aprilTagRefPoses.add(
						imposedPose
					);

					drivetrain.addVisionMeasurement(imposedPose, Timer.getFPGATimestamp());
				}
			}
		}

		Logger.recordOutput(
			"Imposed Translation List", aprilTagRefPoses.toArray(new Pose2d[] {})
		);

	}

	/**
	 * Drive to pose function.
	 * @param target target pose to align to.
	 */
	public void driveToPose(Pose2d target) {

		if (driveToPoseRunning && driveController.atGoal() && thetaController.atGoal()) {
			drivetrain.setControl(brake);
			return;
		}

		if (!driveToPoseRunning) {
			driveToPoseRunning = true;

			Pose2d currentPose = drivetrain.getState().Pose;
			ChassisSpeeds fieldVelocity = drivetrain.getState().Speeds;
			Translation2d linearFieldVelocity =
					new Translation2d(
						fieldVelocity.vxMetersPerSecond,
						fieldVelocity.vyMetersPerSecond
					);
			driveController.reset(
					currentPose.getTranslation().getDistance(target.getTranslation()),
					Math.min(
						0.0,
						-linearFieldVelocity
							.rotateBy(
								target
									.getTranslation()
									.minus(currentPose.getTranslation())
									.getAngle()
									.unaryMinus())
							.getX()));

			thetaController.reset(
				currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
			lastSetpointTranslation = currentPose.getTranslation();
		}

		Pose2d currentPose = drivetrain.getState().Pose;

		// Calculate drive speed
		double currentDistance =
			currentPose.getTranslation().getDistance(target.getTranslation());
		double ffScaler =
			MathUtil.clamp(
				(currentDistance - AutoConstants.FF_MIN_RADIUS)
					/ (AutoConstants.FF_MAX_RADIUS - AutoConstants.FF_MIN_RADIUS),
				0.0,
				1.0);
		driveErrorAbs = currentDistance;
		driveController.reset(
			lastSetpointTranslation.getDistance(target.getTranslation()),
			driveController.getSetpoint().velocity);

		double driveVelocityScalar =
			driveController.getSetpoint().velocity * ffScaler
				+ driveController.calculate(driveErrorAbs, 0.0);

		if (currentDistance < driveController.getPositionTolerance()) {
			driveVelocityScalar = 0.0;
		}

		lastSetpointTranslation =
			new Pose2d(
					target.getTranslation(),
					currentPose.getTranslation().minus(target.getTranslation()).getAngle())
				.transformBy(
					new Transform2d(
						driveController.getSetpoint().position, 0.0,
						new Rotation2d()
					)
				)
				.getTranslation();

		// Calculate theta speed
		double thetaVelocity =
			thetaController.getSetpoint().velocity * ffScaler
				+ thetaController.calculate(
					currentPose.getRotation().getRadians(), target.getRotation().getRadians());

		thetaErrorAbs =
			Math.abs(currentPose.getRotation().minus(target.getRotation()).getRadians());

		if (thetaErrorAbs < thetaController.getPositionTolerance()) {
			thetaVelocity = 0.0;
		}

		Translation2d driveVelocity =
			new Pose2d(
					new Translation2d(),
					currentPose.getTranslation().minus(target.getTranslation()).getAngle())
				.transformBy(
					new Transform2d(
						driveVelocityScalar, 0.0, new Rotation2d()
					)
				)
				.getTranslation();

		// Scale feedback velocities by input ff
		final double linearS = linearFF.get().getNorm() * 3.0;
		final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
		driveVelocity =
			driveVelocity.interpolate(linearFF.get().times(MAX_SPEED), linearS);
		thetaVelocity =
			MathUtil.interpolate(
				thetaVelocity, omegaFF.getAsDouble() * MAX_ANGULAR_RATE, thetaS);

		drivetrain.setControl(
			drive.withVelocityX(
				-driveVelocity.getX()
			).withVelocityY(
				-driveVelocity.getY()
			).withRotationalRate(
				-thetaVelocity
			)
		);

		Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
		Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
		Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
		Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
		Logger.recordOutput(
			"DriveToPose/Setpoint",
			new Pose2d(
				lastSetpointTranslation,
				Rotation2d.fromRadians(thetaController.getSetpoint().position)
		));
		Logger.recordOutput("DriveToPose/Goal", target);
	}

	/**
	 * Handles reef tag alignment by seeing the nearest reef tag.
	 * @param input
	 */
	public void handleReefTagAlignment(TeleopInput input) {

	}

	/**
	 * Handles station tag alignment by aligning with the nearest station tag.
	 * @param input
	 */
	public void handleStationTagAlignment(TeleopInput input) {

	}

	/**
	 * Handle tag alignment state.
	 * @param input
	 * @param id
	 * @param allianceFlip whether or not to invert the controls.
	 * allianceFlip should be true in TeleOp.
	 */
	private void handleTagAlignment(TeleopInput input, int id, boolean allianceFlip) {
		logger.applyStateLogging(drivetrain.getState());

	}

	/**
	* Returns a command that sets the drivetrain to brake mode.
	* @return A command that sets the drivetrain to brake mode.
	*/
	public Command brakeCommand() {
		class BrakeCommand extends Command {
			@Override
			public boolean isFinished() {
				drivetrain.setControl(brake);
				return true;
			}
		}

		return new BrakeCommand();
	}

	/**
	* Command to align to any visible reef tags or not move if none are seen.
	* @param id the id of the tag to align to
	* @param x
	* @param y
	* @return align to tag command.
	*/
	public Command alignToTagCommand(int id, double x, double y) {
		class AlignToReefTagCommand extends Command {

		}
		return new AlignToReefTagCommand();
	}

	/**
	 * Get the maple-Sim Swerve simulation.
	 * @return the simulation
	 */
	public MapleSimSwerveDrivetrain getMapleSimDrivetrain() {
		return drivetrain.getSimDrivetrain();
	}

	/**
	 * Update the raspberry pi simulation state.
	 */
	public void updateRaspberryPi() {
		rpi.update(getMapleSimDrivetrain().getDriveSimulation().getSimulatedDriveTrainPose());
	}
}
