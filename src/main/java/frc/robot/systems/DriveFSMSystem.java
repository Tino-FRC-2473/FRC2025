package frc.robot.systems;


// WPILib Imports
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;


import org.littletonrobotics.junction.Logger;

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
import frc.robot.RaspberryPI;
import frc.robot.AprilTag;

public class DriveFSMSystem extends SubsystemBase {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE,
		ALIGN_TO_TAG_STATE
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
	private RaspberryPI rpi = new RaspberryPI();
	private boolean tagPositionAligned;

	// Placeholder for until confidence system implemented
	private int tagID = AutoConstants.B_REEF_1_TAG_ID;

	private Pose2d tagAlignmentPose = null;

	private Rotation2d rotationAlignmentPose =
		new Rotation2d(0);



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
		//System.out.println("TELEOP-X " + drivetrain.getState().Pose.getX());

		switch (currentState) {
			case TELEOP_STATE:
				handleTeleOpState(input);
				break;
			case ALIGN_TO_TAG_STATE:
				handleTagAlignment(input, tagID, 0, 0);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
		MechLogging.getInstance().
			setDrivePoseData(drivetrain.getState().Pose);
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
					return FSMState.ALIGN_TO_TAG_STATE;
				} else {
					return FSMState.TELEOP_STATE;
				}
			case ALIGN_TO_TAG_STATE:
				if (input.getDriveSquareButton()) {
					return FSMState.ALIGN_TO_TAG_STATE;
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

		/* cv alignment constant resets */
		if (tagAlignmentPose != null) {
			tagAlignmentPose = null;
			tagPositionAligned = false;
		}

		//System.out.println("TELEOP IS RUNNING");

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

		// System.out.println("TELEOP X:" + drivetrain.getState().Pose.getX());

		drivetrain.setControl(
			driveFacingAngle.withVelocityX(xSpeed)
			.withVelocityY(ySpeed)
			.withTargetDirection(
				rotationAlignmentPose
			)
			.withTargetRateFeedforward(rotXComp)
		);

		if (input.getDriveTriangleButton()) {
			drivetrain.setControl(brake);
		}

		if (input.getDriveCircleButton()) {
			drivetrain.setControl(
				point.withModuleDirection(new Rotation2d(-input.getDriveLeftJoystickY(),
					-input.getDriveLeftJoystickX()))
			);
		}

		if (input.getDriveBackButtonPressed()) {
			drivetrain.seedFieldCentric();
		}
	}

	/**
	 * Handle tag alignment state.
	 * @param input
	 * @param id
	 * @param xOff
	 * @param yOff
	 */
	private void handleTagAlignment(TeleopInput input, int id, double xOff, double yOff) {
		logger.applyStateLogging(drivetrain.getState());

		AprilTag tag = rpi.getAprilTagWithID(id);

		System.out.println("Current Pose X" + drivetrain.getState().Pose.getX());
		System.out.println("Current Pose Y" + drivetrain.getState().Pose.getY());

		if (tagAlignmentPose != null) {
			System.out.println("Tag Alignment Pose X " + tagAlignmentPose.getX());
			System.out.println("Tag Alignment Pose Y " + tagAlignmentPose.getY());
		}


		if (tag != null && !tagPositionAligned) {

			Pose2d currPose = drivetrain.getState().Pose;

			// X is forward on robot Pose, z is forward on cv side
			double rpiX = currPose.getX() + tag.getZ();
			// Y is side-to-side on robotPose, x is side-to-side on cv side
			double rpiY = currPose.getY() + tag.getX();
			// using rvec to determine the absolute rotation of the apriltag.
			double rpiTheta = currPose.getRotation().getRadians() + tag.getYaw();

			Pose2d sendPose = new Pose2d(
				rpiX,
				rpiY,
				new Rotation2d(rpiTheta)
			);

			tagPositionAligned = driveToPose(sendPose);
			tagAlignmentPose = sendPose;

			// Logger.recordOutput("Tag Alignment Pose X", tagAlignmentPose.getX());
			// Logger.recordOutput("Tag Alignment Pose Y",tagAlignmentPose.getY());
		} else {
			if (tagPositionAligned) {
				tagAlignmentPose = null;
				drivetrain.setControl(brake);
			}

			if (tagAlignmentPose != null) {
				System.out.println("back up alignment reached");
				tagPositionAligned = driveToPose(tagAlignmentPose);
			}
		}
	}

	private boolean driveToPose(Pose2d targetPose) {
		Pose2d pose = drivetrain.getState().Pose;

		double xDiff = targetPose.getX() - pose.getX();
		double yDiff = targetPose.getY() - pose.getY();
		double aDiff = targetPose.getRotation().getRadians() - pose.getRotation().getRadians();

		System.out.println(
			"XDIFF" + xDiff
		);
		System.out.println(
			"YDIFF" + yDiff
		);
		System.out.println(
			"ADIFF" + aDiff
		);

		Logger.recordOutput("TARGET POSE", targetPose);

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

		Logger.recordOutput("DriveToPose/X Speed", xSpeed);
		Logger.recordOutput("DriveToPose/Y Speed", ySpeed);
		Logger.recordOutput("DriveToPose/A Speed", aSpeed);

		drivetrain.setControl(
			driveFacingAngle.withVelocityX(xSpeed * MAX_SPEED)
			.withVelocityY(-ySpeed * MAX_SPEED)
			.withTargetRateFeedforward(-aSpeed * MAX_ANGULAR_RATE)
			.withTargetDirection(targetPose.getRotation())
		);

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
	 * @param xOffset the x offset for aligning to the tag
	 * @param id the id of the tag to align to
	 * @param yOffset the y offset for aligning to the tag
	 * @return align to tag command.
	 */
	public Command alignToTagCommand(int id, double xOffset, double yOffset) {
		class AlignToReefTagCommand extends Command {

			private int tagID;
			private double xOffset;
			private double yOffset;

			AlignToReefTagCommand(int id, double xo, double yo) {
				this.tagID = id;
				this.xOffset = xo;
				this.yOffset = yo;
				tagPositionAligned = false; //likely redundant.
			}

			@Override
			public void execute() {
				handleTagAlignment(null, this.tagID, this.xOffset, this.yOffset);
			}

			@Override
			public boolean isFinished() {
				return tagPositionAligned;
			}

			@Override
			public void end(boolean interrupted) {
				drivetrain.setControl(brake);
				tagPositionAligned = false;
			}
		}

		return new AlignToReefTagCommand(tagID, xOffset, yOffset);
	}


	/**
	 * Get the maple-Sim Swerve simulation.
	 * @return the simulation
	 */
	public MapleSimSwerveDrivetrain getMapleSimDrivetrain() {
		return drivetrain.getSimDrivetrain();
	}
}
