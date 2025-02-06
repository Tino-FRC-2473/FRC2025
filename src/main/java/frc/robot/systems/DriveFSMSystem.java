package frc.robot.systems;


// WPILib Imports
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

//CTRE Imports
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import choreo.auto.AutoFactory;

import com.ctre.phoenix6.swerve.SwerveRequest;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.simulation.MapleSimSwerveDrivetrain;
import frc.robot.utils.SwerveUtils;
import frc.robot.logging.MechLogging;
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
		.withDeadband(MAX_SPEED * DriveConstants.DRIVE_DEADBAND) // 20% deadband
		.withRotationalDeadband(MAX_ANGULAR_RATE * DriveConstants.ROTATION_DEADBAND) //10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop for drive motors
	private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle
		= new SwerveRequest.FieldCentricFacingAngle()
		.withDeadband(MAX_SPEED * DriveConstants.DRIVE_DEADBAND) // 20% deadband
		.withRotationalDeadband(MAX_ANGULAR_RATE * DriveConstants.ROTATION_DEADBAND) //10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop for drive motors
	private final SwerveRequest.RobotCentric driveRobotCentric
		= new SwerveRequest.RobotCentric()
		.withDeadband(MAX_SPEED * DriveConstants.DRIVE_DEADBAND) // 20% deadband
		.withRotationalDeadband(MAX_ANGULAR_RATE * DriveConstants.ROTATION_DEADBAND) //10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop for drive motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	private final SwerveLogging logger = new SwerveLogging(MAX_SPEED);
	private CommandSwerveDrivetrain drivetrain;

	/* -- cv constants -- */
	private RaspberryPi rpi = new RaspberryPi();
	private boolean tagPositionAligned;
	private int tagID = -1;
	private Pose2d tagAlignmentPose = null;
	private Rotation2d rotationAlignmentPose =
		new Rotation2d(0);

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

	private Comparator<AprilTag> aComparator = new Comparator<AprilTag>() {
		@Override
		public int compare(AprilTag o1, AprilTag o2) {
			return o1.compareTo(o2);
		}
	};

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

		drivetrain.applyOperatorPerspective();

		switch (currentState) {
			case TELEOP_STATE:

				/* cv alignment constant resets */
				if (tagAlignmentPose != null) {
					tagAlignmentPose = null;
					tagPositionAligned = false;
					tagID = -1;
				}

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

		MechLogging.getInstance().
			setDrivePoseData(drivetrain.getState().Pose);

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
				} else if (input.getDriveCircleButton()) {
					return FSMState.ALIGN_TO_STATION_TAG_STATE;
				} else {
					return FSMState.TELEOP_STATE;
				}
			case ALIGN_TO_REEF_TAG_STATE:
				if (input.getDriveSquareButton()) {
					return FSMState.ALIGN_TO_REEF_TAG_STATE;
				} else if (input.getDriveCircleButton()) {
					return FSMState.ALIGN_TO_STATION_TAG_STATE;
				} else {
					return FSMState.TELEOP_STATE;
				}
			case ALIGN_TO_STATION_TAG_STATE:
				if (input.getDriveSquareButton()) {
					return FSMState.ALIGN_TO_REEF_TAG_STATE;
				} else if (input.getDriveCircleButton()) {
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
			input.getDriveLeftJoystickY(), DriveConstants.DRIVE_DEADBAND
		) * MAX_SPEED / DriveConstants.SPEED_DAMP_FACTOR;
		// Drive forward with negative Y (forward) ^

		double ySpeed = -MathUtil.applyDeadband(
			input.getDriveLeftJoystickX(), DriveConstants.DRIVE_DEADBAND
		) * MAX_SPEED / DriveConstants.SPEED_DAMP_FACTOR;
		// Drive left with negative X (left) ^

		double rotXComp = -MathUtil.applyDeadband(
			input.getDriveRightJoystickX(), DriveConstants.DRIVE_DEADBAND
		);
		// Drive left with negative X (left) ^

		if (rotXComp != 0) {
			rotationAlignmentPose = drivetrain.getState().Pose.getRotation();
		}

		drivetrain.setControl(
			driveFacingAngle
				.withVelocityX(xSpeed)
				.withVelocityY(ySpeed)
				.withTargetDirection(
					rotationAlignmentPose
				)
			.withTargetRateFeedforward(rotXComp)
		);

		if (input.getDriveTriangleButton()) {
			drivetrain.setControl(brake);
		}

		if (input.getDriveBackButtonPressed()) {
			drivetrain.seedFieldCentric();
		}
	}

	/**
	 * Handles reef tag alignment by seeing the nearest reef tag.
	 * @param input
	 */
	public void handleReefTagAlignment(TeleopInput input) {

		ArrayList<AprilTag> sortedTagList = rpi.getAprilTags();
		Collections.sort(rpi.getAprilTags(), aComparator);

		if (DriverStation.getAlliance().equals(Alliance.Blue) && tagID == -1) {
			for (AprilTag tag: sortedTagList) {
				for (int id: blueReefTagArray) {
					if (tag.getTagID() == id) {
						tagID = id;
						break;
					}
				}
			}

		} else if (DriverStation.getAlliance().equals(Alliance.Red) && tagID == -1) {
			for (AprilTag tag: sortedTagList) {
				for (int id: redReefTagArray) {
					if (tag.getTagID() == id) {
						tagID = id;
						break;
					}
				}
			}

		}

		if (tagID != -1) {
			handleTagAlignment(input, tagID);
		} else {
			drivetrain.setControl(brake);
		}
	}

	/**
	 * Handles station tag alignment by aligning with the nearest station tag.
	 * @param input
	 */
	public void handleStationTagAlignment(TeleopInput input) {

		ArrayList<AprilTag> sortedTagList = rpi.getAprilTags();
		Collections.sort(rpi.getAprilTags(), aComparator);

		if (DriverStation.getAlliance().equals(Alliance.Blue) && tagID == -1) {
			for (AprilTag tag: sortedTagList) {
				for (int id: blueStationTagArray) {
					if (tag.getTagID() == id) {
						tagID = id;
						break;
					}
				}
			}

		} else if (DriverStation.getAlliance().equals(Alliance.Red) && tagID == -1) {
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
			handleTagAlignment(input, tagID);
		} else {
			drivetrain.setControl(brake);
		}
	}

	/**
	* Handle tag alignment state.
	* @param input
	* @param id
	*/
	private void handleTagAlignment(TeleopInput input, int id) {
		logger.applyStateLogging(drivetrain.getState());

		AprilTag tag = rpi.getAprilTagWithID(id);

		if (tag != null && !tagPositionAligned) {

			Pose2d currPose = drivetrain.getState().Pose;

			// X is forward on robot Pose, z is forward on cv side
			double rpiX = currPose.getX() + tag.getZ();
			// Y is side-to-side on robotPose, x is side-to-side on cv side
			double rpiY = currPose.getY() + tag.getX();
			// Pitch is the value that is coupled with rotation on cv side
			double rpiTheta = currPose.getRotation().getRadians() + tag.getPitch();

			Pose2d sendPose = new Pose2d(
				rpiX,
				rpiY,
				new Rotation2d(rpiTheta)
			);

			tagPositionAligned = driveToPose(sendPose);
			tagAlignmentPose = sendPose;
		} else {
			if (tagPositionAligned) {
				tagAlignmentPose = null;
				drivetrain.setControl(brake);
			}

			if (tagAlignmentPose != null) {
				tagPositionAligned = driveToPose(tagAlignmentPose);
			}
		}
	}

	private boolean driveToPose(Pose2d targetPose) {
		Pose2d pose = drivetrain.getState().Pose;

		double xDiff = targetPose.getX() - pose.getX();
		double yDiff = targetPose.getY() - pose.getY();
		double aDiff = targetPose.getRotation().getRadians() - pose.getRotation().getRadians();

		double xSpeed = Math.abs(xDiff) > VisionConstants.X_MARGIN_TO_REEF
			? SwerveUtils.clamp(
				xDiff / VisionConstants.TRANSLATIONAL_ACCEL_CONSTANT,
				-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
				VisionConstants.MAX_SPEED_METERS_PER_SECOND
			) : 0;
		double ySpeed = Math.abs(yDiff) > VisionConstants.Y_MARGIN_TO_REEF
			? SwerveUtils.clamp(
				yDiff / VisionConstants.TRANSLATIONAL_ACCEL_CONSTANT,
				-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
				VisionConstants.MAX_SPEED_METERS_PER_SECOND
			) : 0;
		double aSpeed = Math.abs(aDiff) > VisionConstants.ROT_MARGIN_TO_REEF
			? SwerveUtils.clamp(
				aDiff / VisionConstants.ROTATIONAL_ACCEL_CONSTANT,
				-VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
				VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND
			) : 0;

		driveFacingAngle.withVelocityX(xSpeed * MAX_SPEED)
			.withVelocityY(-ySpeed * MAX_SPEED)
			.withTargetRateFeedforward(-aSpeed * MAX_ANGULAR_RATE)
			.withTargetDirection(targetPose.getRotation());

		return (xSpeed == 0 && ySpeed == 0 && aSpeed == 0);
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
	* @return align to tag command.
	*/
	public Command alignToTagCommand(int id) {
		class AlignToReefTagCommand extends Command {

			AlignToReefTagCommand(int id) {
				tagID = id;
				tagPositionAligned = false; //likely redundant.
			}

			@Override
			public void execute() {
				handleTagAlignment(null, tagID);
			}

			@Override
			public boolean isFinished() {
				return tagPositionAligned;
			}

			@Override
			public void end(boolean interrupted) {
				drivetrain.setControl(brake);
				tagPositionAligned = false;
				tagID = -1;
				tagAlignmentPose = null;
			}
		}

		return new AlignToReefTagCommand(tagID);
	}

	/**
	 * A Drive Robot Relative Offset command used while extending the elevator in auto.
	 * @param xSpeed x robot relative speed
	 * @param ySpeed y robot relative speed
	 * @param timeRunning the amount of time you should be at the x and y speed for
	 * @return the command
	 */
	public Command driveRobotRelativeOffset(double xSpeed, double ySpeed, double timeRunning) {
		return new DriveRobotRelativeOffsetCommand(xSpeed, ySpeed, timeRunning);
	}

	/**
	 * Return robot relative offset command that drives to left.
	 * @return command
	 */
	public Command driveRobotLeftRelativeOffset() {
		return new DriveRobotRelativeOffsetCommand(
			AutoConstants.REEF_OFFSET_X_AUTO_SPEED,
			-AutoConstants.REEF_OFFSET_Y_AUTO_SPEED,
			AutoConstants.TIME_DRIVING_OFFSET
		);
	}

	/**
	 * Return robot relative offset command that drives to right.
	 * @return command
	 */
	public Command driveRobotRightRelativeOffset() {
		return new DriveRobotRelativeOffsetCommand(
			AutoConstants.REEF_OFFSET_X_AUTO_SPEED,
			AutoConstants.REEF_OFFSET_Y_AUTO_SPEED,
			AutoConstants.TIME_DRIVING_OFFSET);
	}

	class DriveRobotRelativeOffsetCommand extends Command {

		private Timer timer;

		private double xS;
		private double yS;
		private double duration;

		DriveRobotRelativeOffsetCommand(double xSpeed, double ySpeed, double timeRunning) {
			xS = xSpeed;
			yS = ySpeed;
			duration = timeRunning;
			timer = new Timer();
			timer.reset();
		}

		@Override
		public void initialize() {
			timer.start();
		}

		@Override
		public void execute() {
			drivetrain.setControl(
				driveRobotCentric
					.withVelocityX(xS * MAX_SPEED)
					.withVelocityY(yS * MAX_SPEED)
			);
		}

		@Override
		public boolean isFinished() {
			return timer.get() >= duration;
		}

		@Override
		public void end(boolean interrupted) {
			timer.stop();
		}
	}

	/**
	* Get the maple-Sim Swerve simulation.
	* @return the simulation
	*/
	public MapleSimSwerveDrivetrain getMapleSimDrivetrain() {
		return drivetrain.getSimDrivetrain();
	}
}
