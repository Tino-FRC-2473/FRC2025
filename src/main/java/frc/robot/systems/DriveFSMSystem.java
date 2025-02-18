package frc.robot.systems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
// WPILib Imports
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
//CTRE Imports
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;

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
	private Rotation2d rotationAlignmentPose;

	/* -- cv constants -- */
	private RaspberryPi rpi = new RaspberryPi();
	private int tagID = -1;
	private double alignmentYOff;
	private double alignmentXOff;

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
	private ArrayList<Pose2d> aprilTagReefRefPoses;
	private ArrayList<Pose2d> aprilTagStationRefPoses;
	private ArrayList<Pose2d> aprilTagVisionPoses;

	private boolean driveToPoseRunning = false;
	private Timer driveToPoseTimer = new Timer();
	private boolean driveToPoseFinished = false;

	private final PIDController autoXPid = new PIDController(0.3, 0, 0);
	private final PIDController autoYPid = new PIDController(0.3, 0, 0);
	private final ProfiledPIDController autoHeadingPid
		= new ProfiledPIDController(0.75, 0, 0,
			new Constraints(
				AutoConstants.ALIGN_MAX_R_SPEED,
				AutoConstants.ALIGN_MAX_R_ACCEL
			));

	private HolonomicDriveController driveController
		= new HolonomicDriveController(autoXPid, autoYPid, autoHeadingPid);

	private TrajectoryConfig trajConfig;

	private final SwerveRequest.ApplyFieldSpeeds pathApplyFieldSpeeds =
		new SwerveRequest.ApplyFieldSpeeds();

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

		trajConfig = new TrajectoryConfig(
			AutoConstants.ALIGN_MAX_T_SPEED,
			AutoConstants.ALIGN_MAX_T_ACCEL
		)
		.setKinematics(drivetrain.getKinematics())
		.setEndVelocity(0);

		driveController.setTolerance(new Pose2d(
			VisionConstants.X_MARGIN_TO_REEF,
			VisionConstants.Y_MARGIN_TO_REEF,
			new Rotation2d(VisionConstants.ROT_MARGIN_TO_REEF)
		));

		autoHeadingPid.enableContinuousInput(-Math.PI, Math.PI);

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

		/* --- cv alignment reset --- */
		tagID = -1;
		alignmentXOff = 0;
		alignmentYOff = 0;
		driveToPoseRunning = false;
		driveToPoseFinished = false;

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
		aprilTagReefRefPoses = new ArrayList<Pose2d>();
		aprilTagStationRefPoses = new ArrayList<Pose2d>();
		aprilTagVisionPoses = new ArrayList<Pose2d>();
		ArrayList<AprilTag> reefTags = rpi.getReefAprilTags();
		ArrayList<AprilTag> stationTags = rpi.getStationAprilTags();

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

			aprilTagVisionPoses.add(alignmentPose2d);

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

					aprilTagReefRefPoses.add(
						imposedPose
					);

					drivetrain.addVisionMeasurement(imposedPose, Utils.getCurrentTimeSeconds());
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

			aprilTagVisionPoses.add(alignmentPose2d);

			Transform2d robotToCamera =
				new Transform2d(
					new Translation2d(
						SimConstants.ROBOT_TO_STATION_CAMERA.getX(),
						SimConstants.ROBOT_TO_STATION_CAMERA.getY()
					),
					SimConstants.ROBOT_TO_STATION_CAMERA.getRotation()
					.toRotation2d().rotateBy(Rotation2d.k180deg)
				);

			if (!aprilTagPose3d.isEmpty()) {
				//aprilTagPose3d.get().getRotation()
				//	.toRotation2d().rotateBy(new Rotation2d(tag.getPitch()))

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

					aprilTagStationRefPoses.add(
						imposedPose
					);

					drivetrain.addVisionMeasurement(imposedPose, Utils.getCurrentTimeSeconds());
				}
			}
		}

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

	/**
	 * Drive to pose function.
	 * @param target target pose to align to.
	 */
	public void driveToPose(Pose2d target) {

		if (driveToPoseFinished) {
			drivetrain.setControl(brake);
			return;
		}

		if (!driveToPoseRunning) {
			driveToPoseRunning = true;

			driveToPoseTimer.reset();
			driveToPoseTimer.start();
		}

		Pose2d currPose = (Utils.isSimulation())
			? getMapleSimDrivetrain().getDriveSimulation().getSimulatedDriveTrainPose()
			: drivetrain.getState().Pose;

		driveToPoseFinished =
			Math.abs(target.getX() - currPose.getX()) < VisionConstants.X_MARGIN_TO_REEF
			&& Math.abs(target.getY() - currPose.getY()) < VisionConstants.Y_MARGIN_TO_REEF
			&& Math.abs(
				target.getRotation().getRadians() - currPose.getRotation().getRadians()
			) < VisionConstants.ROT_MARGIN_TO_REEF;

		double xSpeed =
			-(target.getX() - currPose.getX())
			* AutoConstants.ALIGN_DRIVE_P
			* MAX_SPEED * allianceOriented.getAsInt();

		double ySpeed =
			-(target.getY() - currPose.getY())
			* AutoConstants.ALIGN_DRIVE_P
			* MAX_SPEED * allianceOriented.getAsInt();

		double rotSpeed =
			-(target.getRotation().getRadians()
			- currPose.getRotation().getRadians())
			* AutoConstants.ALIGN_THETA_P
			* MAX_ANGULAR_RATE;


		Logger.recordOutput(
			"DriveToPose/Pose", currPose
		);
		Logger.recordOutput(
			"DriveToPose/Time", driveToPoseTimer.get()
		);
		Logger.recordOutput(
			"DriveToPose/IsFinished", driveToPoseFinished
		);
		Logger.recordOutput(
			"DriveToPose/IsRunning", driveToPoseRunning
		);

		Logger.recordOutput(
			"DriveToPose/XSpeed", xSpeed
		);
		Logger.recordOutput(
			"DriveToPose/YSpeed", ySpeed
		);
		Logger.recordOutput(
			"DriveToPose/RotSpeed", rotSpeed
		);

		Logger.recordOutput(
			"DriveToPose/RotDiff",
				(target.getRotation().getRadians() - currPose.getRotation().getRadians())
		);
		Logger.recordOutput(
			"DriveToPose/XDiff",
				(target.getX() - currPose.getX())
		);
		Logger.recordOutput(
			"DriveToPose/YDiff",
				(target.getY() - currPose.getY())
		);


		drivetrain.setControl(
			driveFacingAngle
			.withVelocityX(xSpeed)
			.withVelocityY(ySpeed)
			.withTargetRateFeedforward(-rotSpeed)
			.withTargetDirection(target.getRotation())
		);
	}

	/**
	 * Handles reef tag alignment by seeing the nearest reef tag.
	 * @param input
	 */
	public void handleReefTagAlignment(TeleopInput input) {

		if (input != null) {
			if (input.getDriveLeftBumperButton()) {
				alignmentYOff = AutoConstants.REEF_Y_L_TAG_OFFSET;
			} else if (input.getDriveRightBumperButton()) {
				alignmentYOff = AutoConstants.REEF_Y_R_TAG_OFFSET;
			} else {
				alignmentYOff = 0;
			}
		}

		alignmentXOff = AutoConstants.REEF_X_TAG_OFFSET;

		ArrayList<AprilTag> sortedTagList = rpi.getReefAprilTags();
		Collections.sort(sortedTagList, aComparator);

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

		if (tagID != -1) {
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
			if (input.getDriveLeftBumperButton()) {
				alignmentYOff = AutoConstants.STATION_Y_L_TAG_OFFSET;
			} else if (input.getDriveRightBumperButton()) {
				alignmentYOff = AutoConstants.STATION_Y_R_TAG_OFFSET;
			} else {
				alignmentYOff = 0;
			}
		}

		ArrayList<AprilTag> sortedTagList = rpi.getStationAprilTags();
		Collections.sort(sortedTagList, aComparator);

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

		if (tagID != -1) {
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
		logger.applyStateLogging(drivetrain.getState());

		Optional<Pose3d> tagAbsPose = aprilTagFieldLayout.getTagPose(id);
		if (!tagAbsPose.isEmpty()) {
			Pose2d alignmentPose = tagAbsPose
				.get().toPose2d()
				.transformBy(
				new Transform2d(
					alignmentXOff,
					alignmentYOff,
					Rotation2d.k180deg
				)
			);

			driveToPose(alignmentPose);

			Logger.recordOutput("AprilTag ID", id);
			Logger.recordOutput("Alignment Pose", alignmentPose);
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
