package frc.robot.systems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
// WPILib Imports
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
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
import java.util.Arrays;
import java.util.Optional;
import java.util.function.IntSupplier;

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
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.HardwareMap;
import frc.robot.RaspberryPi;
import frc.robot.RaspberryPiPhoton;
import frc.robot.AprilTag;

public class DriveFSMSystem extends SubsystemBase {
	/* ======================== Constants ======================== */

	// FSM state definitions
	public enum DriveFSMState {
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
	private final SwerveRequest.ApplyFieldSpeeds pathApplyFieldSpeeds =
		new SwerveRequest.ApplyFieldSpeeds();

	private CommandSwerveDrivetrain drivetrain;
	private Rotation2d rotationAlignmentPose;
	private	Pose2d alignmentPose2d = null;
	private boolean driveToPoseRunning = false;

	/* -- cv constants -- */
	private RaspberryPi rpi = new RaspberryPi();
	private int tagID = -1;
	private double alignmentYOff;
	private double alignmentXOff;

	private ArrayList<Pose2d> aprilTagReefRefPoses = new ArrayList<Pose2d>();
	private ArrayList<Pose2d> aprilTagStationRefPoses = new ArrayList<Pose2d>();
	private ArrayList<Pose2d> aprilTagVisionPoses = new ArrayList<Pose2d>();
	private AprilTagFieldLayout aprilTagFieldLayout;
	private boolean hasLocalized = false;

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
		if (DriverStation.getAlliance().isEmpty()) {
			return -1;
		}
		return DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1;
	};

	private SlewRateLimiter slewRateX;
	private SlewRateLimiter slewRateY;

	private ElevatorFSMSystem elevatorSystem;

	private boolean driveToPoseFinished = false;
	private boolean aligningToReef = false;
		// False => aligning to station, True => aligning to reef

	private final ProfiledPIDController driveController = new ProfiledPIDController(
		AutoConstants.ALIGN_DRIVE_P, 0, 0, new TrapezoidProfile.Constraints(
			AutoConstants.ALIGN_MAX_T_SPEED, AutoConstants.ALIGN_MAX_T_ACCEL
		)
	);

	private final ProfiledPIDController thetaController = new ProfiledPIDController(
		AutoConstants.ALIGN_THETA_P, 0, 0, new TrapezoidProfile.Constraints(
			AutoConstants.ALIGN_MAX_R_SPEED, AutoConstants.ALIGN_MAX_R_ACCEL
		)
	);

	private double driveErrorAbs;
	private double thetaErrorAbs;
	private Translation2d lastSetpointTranslation;

	/* ======================== Private variables ======================== */
	private DriveFSMState currentState;

	/* ======================== Constructor ======================== */
	/**
	 * Create DriveFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 * @param elevatorFSMSystem The ElevatorFSMSystem instance to be used by this system.
	 */
	public DriveFSMSystem(ElevatorFSMSystem elevatorFSMSystem) {
		// Perform hardware init
		drivetrain = TunerConstants.createDrivetrain();
		rpi = (Utils.isSimulation()) ? new RaspberryPiSim() : new RaspberryPiPhoton();

		slewRateX = new SlewRateLimiter(DriveConstants.SLEW_RATE);
		slewRateY = new SlewRateLimiter(DriveConstants.SLEW_RATE);

		if (elevatorFSMSystem != null) {
			elevatorSystem = elevatorFSMSystem;
		} else {
			elevatorSystem = null;
		}

		try {
			aprilTagFieldLayout
				= new AprilTagFieldLayout(VisionConstants.APRIL_TAG_FIELD_LAYOUT_JSON);
		} catch (IOException e) {
			e.printStackTrace();
		}

		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		// Reset state machine
		reset();
	}

	/**
	 * Default constructor for DriveFSMSystem.
	 */
	public DriveFSMSystem() {
		this(null);
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public DriveFSMState getCurrentState() {
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
		currentState = DriveFSMState.TELEOP_STATE;
		rotationAlignmentPose =
			(Utils.isSimulation())
				? getMapleSimDrivetrain().getDriveSimulation()
				.getSimulatedDriveTrainPose().getRotation()
				: drivetrain.getState().Pose.getRotation();

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
			case TELEOP_STATE -> handleTeleOpState(input);
			case ALIGN_TO_REEF_TAG_STATE -> handleReefTagAlignment(input);
			case ALIGN_TO_STATION_TAG_STATE -> handleStationTagAlignment(input);
			default -> throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		if (!HardwareMap.useSuperStructure()) {
			currentState = nextState(input);
		}
	}

	/**
	 * Handles the override state based on the given input.
	 * @param input The TeleopInput instance containing the current input state.
	 */
	public void handleOverrideState(TeleopInput input) {
		currentState = nextState(input);
	}

	/**
	 * Performs specific action based on the autoState passed in.
	 */
	public void updateAutonomous() {
		// if (Utils.isSimulation()) {
		// 	getMapleSimDrivetrain().resetSimulationPose();
		// }
		rpi.printRawData();
	}

	/**
	 * Configures Auto Builder for pathfinding.
	 * @return An AutoFactory instance for creating autonomous routines.
	 */
	public AutoFactory configureAutoSettings() {
		return new AutoFactory(
			() -> drivetrain.getState().Pose,
			drivetrain::resetPose,
			drivetrain::followTrajectory,
			true,
			this
		);
	}

	/**
	* Gets robot alignment status (for LEDs).
	* @return Whether the robot is aligned to the target apriltag.
	*/
	public boolean isAlignedToTag() {
		return driveToPoseFinished;
	}

	/**
	 * Sets the current state of the FSM.
	 * @param state The new state to set.
	 */
	public void setState(DriveFSMState state) {
		currentState = state;
	}

		/**
	 * Update vision measurements according to all seen tags.
	 */
	public void updateVisionEstimates() {
		aprilTagReefRefPoses = new ArrayList<Pose2d>();
		aprilTagStationRefPoses = new ArrayList<Pose2d>();
		aprilTagVisionPoses = new ArrayList<Pose2d>();
		ArrayList<AprilTag> reefTags = rpi.getReefAprilTags();
		ArrayList<AprilTag> stationTags = rpi.getStationAprilTags();

		System.out.println("REEF: " + reefTags.toString());
		System.out.println("STATION: " + stationTags.toString());

		Pose2d currPose;

		if (Utils.isSimulation()) {
			currPose = getMapleSimDrivetrain().getDriveSimulation().getSimulatedDriveTrainPose();
		} else {
			currPose = drivetrain.getState().Pose;
		}

		for (int t = 0; t < reefTags.size(); t++) {
			AprilTag tag = reefTags.get(t);

			Optional<Pose3d> aprilTagPose3d = aprilTagFieldLayout.getTagPose(tag.getTagID());

			Transform2d robotToCamera =
				new Transform2d(
					-SimConstants.ROBOT_TO_REEF_CAMERA.getTranslation().getX(),
						// - if u use pose rotation.
					-SimConstants.ROBOT_TO_REEF_CAMERA.getTranslation().getY(),
						// - if u use pose rotation.
					SimConstants.ROBOT_TO_REEF_CAMERA.getRotation().toRotation2d()
					//.rotateBy(Rotation2d.k180deg)
				);

			if (!aprilTagPose3d.isEmpty()) {
				Pose2d imposedPose =
					aprilTagPose3d.get().toPose2d()
					.transformBy(
						new Transform2d(
							tag.getZ(),
							tag.getX(),
							new Rotation2d()
						)
					).rotateAround(aprilTagPose3d.get().toPose2d().getTranslation(),
						new Rotation2d(tag.getPitch()))
					.transformBy(robotToCamera.inverse());

				aprilTagReefRefPoses.add(
					imposedPose
				);

				if (!hasLocalized) {
					drivetrain.addVisionMeasurement(imposedPose, Utils.getCurrentTimeSeconds());
				} else {
					if (visionEstimateFilter(imposedPose, currPose)) {
						drivetrain.addVisionMeasurement(imposedPose, Utils.getCurrentTimeSeconds());
					}
				}
				hasLocalized = true;

			}
		}

		// for (int t = 0; t < stationTags.size(); t++) {
		// 	AprilTag tag = stationTags.get(t);

		// 	Optional<Pose3d> aprilTagPose3d = aprilTagFieldLayout.getTagPose(tag.getTagID());

		// 	Transform2d robotToCamera =
		// 		new Transform2d(
		// 			new Translation2d(
		// 				SimConstants.ROBOT_TO_STATION_CAMERA.getX(),
		// 				SimConstants.ROBOT_TO_STATION_CAMERA.getY()
		// 			),
		// 			SimConstants.ROBOT_TO_STATION_CAMERA.getRotation()
		// 			.toRotation2d().rotateBy(Rotation2d.k180deg)
		// 		);

		// 	Pose2d alignmentPose = currPose
		// 		.transformBy(robotToCamera)
		// 		.plus(new Transform2d(
		// 			-tag.getZ(),
		// 			(tag.getX()),
		// 			new Rotation2d(-tag.getPitch())))
		// 		.transformBy(robotToCamera.inverse());

		// 	aprilTagVisionPoses.add(alignmentPose);

		// 	if (!aprilTagPose3d.isEmpty()) {

		// 		Pose2d imposedPose = new Pose2d(
		// 			new Pose3d(currPose)
		// 				.plus(aprilTagPose3d.get().minus(new Pose3d(alignmentPose)))
		// 				.toPose2d().getTranslation(),
		// 			aprilTagPose3d.get().getRotation()
		// 				.toRotation2d().rotateBy(new Rotation2d(tag.getPitch() / 2))
		// 		).transformBy(
		// 			robotToCamera.inverse()
		// 		);

		// 		aprilTagStationRefPoses.add(
		// 			imposedPose
		// 		);

		// 		drivetrain.addVisionMeasurement(imposedPose, Utils.getCurrentTimeSeconds());
		// 	}
		// }

		Logger.recordOutput(
			"VisionEstimate/ImposedReefList", aprilTagReefRefPoses.toArray(new Pose2d[] {})
		);
		Logger.recordOutput(
			"VisionEstimate/ImposedStationList", aprilTagStationRefPoses.toArray(new Pose2d[] {})
		);
		Logger.recordOutput(
			"VisionEstimate/AllVisionTargets", aprilTagVisionPoses.toArray(new Pose2d[] {})
		);

	}

	private boolean visionEstimateFilter(Pose2d imposed, Pose2d current) {
		return
			(imposed.getTranslation().getDistance(current.getTranslation())
				< VisionConstants.LOCALIZATION_TRANSLATIONAL_THRESHOLD
			&& Math.abs(imposed.getRotation().getRadians() - current.getRotation().getRadians())
				< VisionConstants.LOCALIZATION_ANGLE_TOLERANCE);
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
	private DriveFSMState nextState(TeleopInput input) {

		switch (currentState) {
			case TELEOP_STATE:
				if (input.getAlignReefButton()) {
					return DriveFSMState.ALIGN_TO_REEF_TAG_STATE;
				}  else if (input.getDriveTriangleButton()) {
					return DriveFSMState.ALIGN_TO_STATION_TAG_STATE;
				} else {
					return DriveFSMState.TELEOP_STATE;
				}
			case ALIGN_TO_REEF_TAG_STATE:
				if (input.getAlignReefButton()) {
					return DriveFSMState.ALIGN_TO_REEF_TAG_STATE;
				}  else if (input.getDriveTriangleButton()) {
					return DriveFSMState.ALIGN_TO_STATION_TAG_STATE;
				} else {
					return DriveFSMState.TELEOP_STATE;
				}
			case ALIGN_TO_STATION_TAG_STATE:
				if (input.getAlignReefButton()) {
					return DriveFSMState.ALIGN_TO_REEF_TAG_STATE;
				}  else if (input.getDriveTriangleButton()) {
					return DriveFSMState.ALIGN_TO_STATION_TAG_STATE;
				} else {
					return DriveFSMState.TELEOP_STATE;
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

		/* --- cv alignment reset --- */
		tagID = -1;
		alignmentXOff = 0;
		alignmentYOff = 0;
		driveToPoseFinished = false;
		driveToPoseRunning = false;

		double constantDamp = 1;

		if (elevatorSystem != null) {
			constantDamp = (elevatorSystem.isElevatorAtL4() || input.getDriveCrossButton())
				? DriveConstants.SPEED_DAMP_FACTOR : DriveConstants.NORMAL_DAMP;
		}

		double xSpeed = MathUtil.applyDeadband(
			slewRateX.calculate(input.getDriveLeftJoystickY()), DriveConstants.JOYSTICK_DEADBAND
			) * MAX_SPEED / constantDamp;
			// Drive forward with negative Y (forward) ^

		double ySpeed = MathUtil.applyDeadband(
			slewRateY.calculate(input.getDriveLeftJoystickX()), DriveConstants.JOYSTICK_DEADBAND
			) * MAX_SPEED / constantDamp;
			// Drive left with negative X (left) ^

		double rotXComp = MathUtil.applyDeadband(
			input.getDriveRightJoystickX(), DriveConstants.JOYSTICK_DEADBAND)
			* MAX_ANGULAR_RATE / constantDamp;
			// Drive left with negative X (left) ^

		if (rotXComp != 0) {
			rotationAlignmentPose =
				(Utils.isSimulation())
					? getMapleSimDrivetrain().getDriveSimulation()
					.getSimulatedDriveTrainPose().getRotation()
					: drivetrain.getState().Pose.getRotation();
		}

		if (!input.getDriveCircleButton()) {
			drivetrain.setControl(
				driveFacingAngle.withVelocityX(xSpeed * allianceOriented.getAsInt())
				.withVelocityY(ySpeed * allianceOriented.getAsInt())
				.withTargetDirection(rotationAlignmentPose)
				.withTargetRateFeedforward(-rotXComp)
				.withHeadingPID(DriveConstants.HEADING_P, 0, 0)
			);
		} else {
			drivetrain.setControl(
				driveRobotCentric.withVelocityX(xSpeed * allianceOriented.getAsInt())
				.withVelocityY(ySpeed * allianceOriented.getAsInt())
				.withRotationalRate(-rotXComp)
			);
		}

		if (input.getSeedGyroButtonPressed()) {
			drivetrain.seedFieldCentric();
			rotationAlignmentPose = new Rotation2d();
			hasLocalized = false;
		}

		Logger.recordOutput("TeleOp/XSpeed", xSpeed);
		Logger.recordOutput("TeleOp/YSpeed", ySpeed);
		Logger.recordOutput("TeleOp/RotSpeed", rotXComp);
	}

	/**
	 * updates drivetrain logging.
	 */
	public void updateLogging() {
		Logger.recordOutput("DriveState/RobotPose", drivetrain.getState().Pose);
		Logger.recordOutput("DriveState/RobotRot", drivetrain.getPigeon2().getYaw().getValue());
		Logger.recordOutput("DriveState/CurrentSwerveStates", drivetrain.getState().ModuleStates);
		Logger.recordOutput("DriveState/TargetSwerveStates", drivetrain.getState().ModuleTargets);
		Logger.recordOutput("DriveState/CurrentChassisSpeeds", drivetrain.getState().Speeds);
	}

	private Timer alignmentTimer = new Timer();

	/**
	 * Drive to pose function.
	 * @param target target pose to align to.
	 * @param allianceFlip whether to incorporate an alliance multiplier in alignment directions.
	 * @return whether or not driving is completed.
	 */
	public boolean driveToPose(Pose2d target, boolean allianceFlip) {
		Pose2d currPose = (Utils.isSimulation())
			? getMapleSimDrivetrain().getDriveSimulation().getSimulatedDriveTrainPose()
			: drivetrain.getState().Pose;

		if (!driveToPoseRunning) {
			driveToPoseRunning = true;
			alignmentTimer.start();

			ChassisSpeeds speeds = (Utils.isSimulation())
				? getMapleSimDrivetrain().getDriveSimulation()
					.getDriveTrainSimulatedChassisSpeedsFieldRelative()
				: drivetrain.getState().Speeds;

			driveController.reset(
				currPose.getTranslation().getDistance(target.getTranslation()),
				Math.min(
					0.0,
					-new Translation2d(
						speeds.vxMetersPerSecond,
						speeds.vyMetersPerSecond
					).rotateBy(
						target.getTranslation()
						.minus(currPose.getTranslation())
						.getAngle()
						.unaryMinus()
					).getX()
				)
			);

			thetaController.reset(currPose.getRotation().getRadians(),
				speeds.omegaRadiansPerSecond);
			lastSetpointTranslation = currPose.getTranslation();
		}

		double currDistance = currPose.getTranslation().getDistance(target.getTranslation());
		double ffScaler = MathUtil.clamp(
			(currDistance - AutoConstants.FF_MIN_RADIUS)
				/ (AutoConstants.FF_MAX_RADIUS - AutoConstants.FF_MIN_RADIUS),
			0.0,
			1.0
		);

		driveErrorAbs = currDistance;

		driveController.reset(
			lastSetpointTranslation.getDistance(target.getTranslation()),
			driveController.getSetpoint().velocity
		);

		double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
			+ driveController.calculate(driveErrorAbs, 0.0);
		if (currDistance < driveController.getPositionTolerance()) {
			driveVelocityScalar = 0.0;
		}

		lastSetpointTranslation = new Pose2d(
			target.getTranslation(),
			currPose.getTranslation().minus(target.getTranslation()).getAngle()
		).transformBy(
			new Transform2d(
				new Translation2d(driveController.getSetpoint().position, 0.0),
				new Rotation2d()
			)
		).getTranslation();

		// Calculate theta speed
		double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
			+ thetaController.calculate(
				currPose.getRotation().getRadians(), target.getRotation().getRadians()
		);
		thetaErrorAbs = Math.abs(
			currPose.getRotation().minus(target.getRotation()).getRadians()
		);

		if (thetaErrorAbs < thetaController.getPositionTolerance()) {
			thetaVelocity = 0.0;
		}

		// Command speeds
		var driveVelocity = new Pose2d(
			new Translation2d(),
			currPose.getTranslation().minus(target.getTranslation())
			.getAngle()
		).transformBy(
			new Transform2d(
				new Translation2d(driveVelocityScalar, 0.0),
				new Rotation2d()
			)
		).getTranslation();

		drivetrain.setControl(
			driveFacingAngle.
				withVelocityX(
					driveVelocity.getX()
				)
				.withVelocityY(
					driveVelocity.getY()
				)
				.withTargetRateFeedforward(thetaVelocity)
				.withTargetDirection(target.getRotation())
				.withHeadingPID(DriveConstants.DRIVE_TO_POSE_HEADING_P, 0, 0)

		);
		//drivetrain.setControl(brake);

		rotationAlignmentPose = currPose.getRotation();

		driveToPoseFinished = driveController.atGoal() && thetaController.atGoal();

		if (driveToPoseFinished) {
			alignmentTimer.stop();
			alignmentTimer.reset();
			drivetrain.setControl(brake);
		}

		Logger.recordOutput("DriveToPose/DriveError", driveErrorAbs);
		Logger.recordOutput("DriveToPose/ThetaError", thetaErrorAbs);
		Logger.recordOutput("DriveToPose/DriveVelocity", driveVelocityScalar);
		Logger.recordOutput("DriveToPose/ThetaVelocity", thetaVelocity);
		Logger.recordOutput("DriveToPose/DriveFinished", driveToPoseFinished);
		Logger.recordOutput("DriveToPose/DriveSetpoint", driveController.getSetpoint().position);
		Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
		Logger.recordOutput("DriveToPose/Time", alignmentTimer.get());
		Logger.recordOutput("DriveToPose/TargetPose", target);

		return driveToPoseFinished;
	}

	/**
	 * Handles reef tag alignment by seeing the nearest reef tag.
	 * @param input
	 */
	public void handleReefTagAlignment(TeleopInput input) {

		if (input != null) {
			if (input.getAlignLeftOffsetButton()) {
				alignmentYOff = AutoConstants.REEF_Y_L_TAG_OFFSET;
			} else if (input.getAlignRightOffsetButton()) {
				alignmentYOff = AutoConstants.REEF_Y_R_TAG_OFFSET;
			} else {
				alignmentYOff = AutoConstants.REEF_Y_L_TAG_OFFSET;
			}
		}

		alignmentXOff = AutoConstants.REEF_X_TAG_OFFSET;

		ArrayList<AprilTag> sortedTagList = rpi.getReefAprilTags();

		if (DriverStation.getAlliance().get().equals(Alliance.Blue) && tagID == -1) {
			for (AprilTag tag: sortedTagList) {
				if (tagID == -1) {
					for (int id: blueReefTagArray) {
						if (tag.getTagID() == id) {
							tagID = id;
							break;
						}
					}
				} else {
					break;
				}
			}

		} else if (DriverStation.getAlliance().get().equals(Alliance.Red) && tagID == -1) {
			for (AprilTag tag: sortedTagList) {
				if (tagID == -1) {
					for (int id: redReefTagArray) {
						if (tag.getTagID() == id) {
							tagID = id;
							break;
						}
					}
				} else {
					break;
				}
			}

		}

		Logger.recordOutput("TagID", tagID);

		if (tagID != -1) {
			aligningToReef = true;
			handleTagAlignment(input, tagID, true);
		} else {
			drivetrain.setControl(brake);
		}
	}

	/**
	 * Handles station tag alignment by aligning with the nearest station tag.
	 * @param input
	 */
	public void handleStationTagAlignment(TeleopInput input) {

		if (input != null) {
			if (input.getAlignLeftOffsetButton()) {
				alignmentYOff = AutoConstants.STATION_Y_L_TAG_OFFSET;
			} else if (input.getAlignRightOffsetButton()) {
				alignmentYOff = AutoConstants.STATION_Y_R_TAG_OFFSET;
			} else {
				alignmentYOff = 0;
			}
		}

		alignmentXOff = -AutoConstants.SOURCE_X_OFFSET;

		ArrayList<AprilTag> sortedTagList = rpi.getStationAprilTags();

		if (DriverStation.getAlliance().get().equals(Alliance.Blue) && tagID == -1) {
			for (AprilTag tag: sortedTagList) {
				for (int id: blueStationTagArray) {
					if (tag.getTagID() == id) {
						tagID = id;
						break;
					}
				}
			}

		} else if (DriverStation.getAlliance().get().equals(Alliance.Red) && tagID == -1) {
			for (AprilTag tag: sortedTagList) {
				for (int id: redStationTagArray) {
					if (tag.getTagID() == id) {
						tagID = id;
						break;
					}
				}
			}

		}

		Logger.recordOutput("TagID", tagID);

		if (tagID != -1) {
			aligningToReef = false;
			handleTagAlignment(input, tagID, true);
		} else {
			drivetrain.setControl(brake);
		}
	}

	/**
	 * Handle tag alignment state.
	 * @param input
	 * @param id
	 * @param allianceFlip whether or not to invert the controls.
	 * allianceFlip should be true in TeleOp.
	 */
	private void handleTagAlignment(TeleopInput input, int id, boolean allianceFlip) {
		AprilTag tag = rpi.getAprilTagWithID(id);
		Pose2d currPose;

		if (Utils.isSimulation()) {
			currPose = getMapleSimDrivetrain().getDriveSimulation().getSimulatedDriveTrainPose();
		} else {
			currPose = drivetrain.getState().Pose;
		}

		Transform3d robotToCamera;
		if (aligningToReef) {
		//TODO: make a reef and station alignment hepler function instead of just one.
			// robotToCamera =
			// new Transform2d(
			// 	SimConstants.ROBOT_TO_REEF_CAMERA.getTranslation().getX(),
			// 		// - if u use pose rotation.
			// 	SimConstants.ROBOT_TO_REEF_CAMERA.getTranslation().getY(),
			// 		// - if u use pose rotation.
			// 	SimConstants.ROBOT_TO_REEF_CAMERA.getRotation().toRotation2d()
			// );
			robotToCamera = SimConstants.ROBOT_TO_REEF_CAMERA;
		} else {
			robotToCamera =
			new Transform3d(
				SimConstants.ROBOT_TO_STATION_CAMERA.getTranslation(),
				SimConstants.ROBOT_TO_STATION_CAMERA.getRotation()
				.rotateBy(new Rotation3d(Rotation2d.k180deg))
			);
			//robotToCamera = SimConstants.ROBOT_TO_STATION_CAMERA;
		}

		System.out.println("TAG Reached here");

		if (tag != null) {
			if (Utils.isSimulation()) {
				alignmentPose2d =
					new Pose3d(
						currPose
					)
					.transformBy(robotToCamera)
					.plus(new Transform3d(
						tag.getZ(),
						(tag.getX()),
						0.0,
						new Rotation3d(
							new Rotation2d(-tag.getPitch())
						)
					))
					.transformBy(robotToCamera.inverse())
					.toPose2d()
					.transformBy(
						new Transform2d(
							-alignmentXOff,
							-alignmentYOff,
							new Rotation2d()
						)
					);

			} else {
				alignmentPose2d =
					new Pose3d(
						currPose
					)
					.transformBy(robotToCamera)
					.plus(new Transform3d(
						tag.getZ(),
						(tag.getX()),
						0.0,
						new Rotation3d(
							new Rotation2d(-tag.getPitch())
						)
					))
					.transformBy(robotToCamera.inverse())
					.toPose2d()
					.transformBy(
						new Transform2d(
							-alignmentXOff,
							-alignmentYOff,
							new Rotation2d()
						)
					);
			}
		}

		if (alignmentPose2d != null) {
			driveToPose(alignmentPose2d, allianceFlip);
		}

		if (driveToPoseFinished || alignmentPose2d == null) {
			drivetrain.setControl(
				drive.withVelocityX(0)
				.withVelocityY(0)
				.withRotationalRate(0)
			);
			return;
		}

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
		class AlignToTagCommand extends Command {

			private Timer alignmentTimerAutoCommand;

			AlignToTagCommand() {
				alignmentTimerAutoCommand = new Timer();
			}

			public void initialize() {
				alignmentXOff = x;
				alignmentYOff = y;

				if (
					Arrays.stream(redReefTagArray).anyMatch(val -> val == id)
					|| Arrays.stream(blueReefTagArray).anyMatch(val -> val == id)
				) {
					aligningToReef = true;
				} else {
					aligningToReef = false;
				}

				alignmentTimerAutoCommand.start();

			}

			@Override
			public void execute() {
				handleTagAlignment(null, id, false);
			}

			public boolean isFinished() {
				return driveToPoseFinished || alignmentTimerAutoCommand.get() > 2;
			}

			public void end(boolean interrupted) {
				tagID = -1;
				alignmentXOff = 0;
				alignmentYOff = 0;
				driveToPoseFinished = false;
				driveToPoseRunning = false;
				alignmentPose2d = null;

				alignmentTimerAutoCommand.stop();
				alignmentTimerAutoCommand.reset();
			}
		}

		return new AlignToTagCommand();
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
