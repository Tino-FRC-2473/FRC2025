package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.time.Year;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
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
import frc.robot.constants.TunerConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.simulation.MapleSimSwerveDrivetrain;
import frc.robot.simulation.RaspberryPiSim;
import frc.robot.utils.SwerveUtils;
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
	private boolean tagPositionAligned;
	private int tagID = -1;
	private Pose2d alignmentPose2d;
	private boolean tagAlignedRotation;
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

	private Comparator<AprilTag> aComparator = new Comparator<AprilTag>() {
		@Override
		public int compare(AprilTag o1, AprilTag o2) {
			return o1.compareTo(o2);
		}
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
		slewRateX = new SlewRateLimiter(DriveConstants.SLEW_RATE, -DriveConstants.SLEW_RATE, 0);
		slewRateY = new SlewRateLimiter(DriveConstants.SLEW_RATE, -DriveConstants.SLEW_RATE, 0);
		slewRateA = new SlewRateLimiter(DriveConstants.SLEW_RATE, -DriveConstants.SLEW_RATE, 0);
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

		//rpi.printRawData();

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

		/* cv alignment constant resets */
		alignmentPose2d = null;
		tagAlignedRotation = false;
		tagPositionAligned = false;
		tagID = -1;

		//System.out.println("TELEOP IS RUNNING");

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
			driveFacingAngle.withVelocityX(
				xSpeed * allianceOriented.getAsInt()
			)
			.withVelocityY(
				ySpeed * allianceOriented.getAsInt()
			)
			.withTargetDirection(
				rotationAlignmentPose
			)
			.withTargetRateFeedforward(
				rotXComp
			)
		);

		if (input.getDriveCircleButton()) {
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

		if (input != null) {
			if (input.getDriveLeftBumperButton()) {
				alignmentYOff = AutoConstants.REEF_Y_L_TAG_OFFSET;
			} else if (input.getDriveRightBumperButton()) {
				alignmentYOff = AutoConstants.REEF_Y_R_TAG_OFFSET;
			} else {
				alignmentYOff = 0;
			}
		}

		ArrayList<AprilTag> sortedTagList = rpi.getReefAprilTags();
		Collections.sort(rpi.getAprilTags(), aComparator);

		System.out.println("reef state reached");

		Logger.recordOutput("AprilTags", (sortedTagList).toString());

		if (DriverStation.getAlliance().get().equals(Alliance.Blue) && tagID == -1) {
			for (AprilTag tag: sortedTagList) {
				System.out.println("TAG ID: " + tag.getTagID());
				if (tagID == -1) {
					for (int id: blueReefTagArray) {
						if (tag.getTagID() == id) {
							System.out.println("TAG ID MATCHED!!: " + tag.getTagID());
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
				System.out.println("TAG ID: " + tag.getTagID());
				if (tagID == -1) {
					for (int id: redReefTagArray) {
						if (tag.getTagID() == id) {
							System.out.println("TAG ID MATCHED!!: " + tag.getTagID());
							tagID = id;
							break;
						}
					}
				} else {
					break;
				}
			}

		}

		//System.out.println("REEF TAG ID: " + tagID);

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
		Collections.sort(rpi.getAprilTags(), aComparator);

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

		AprilTag tag = rpi.getAprilTagWithID(id);

		Logger.recordOutput("APRILTAG ID", id);
		Logger.recordOutput("APRILTAG", tag != null);
		Logger.recordOutput("ROT ALIGNED", tagAlignedRotation);
		Logger.recordOutput("POS ALIGNED", tagPositionAligned);
		Logger.recordOutput("Alignment Pose 2d", alignmentPose2d);

		//handle if the tag's x, y, and rot position is aligned.
		if (tagPositionAligned) {
			//reset odometry to tag abs position, w/ calculated offset.
			drivetrain.setControl(
				driveRobotCentric.withVelocityX(
					DriveConstants.PASSIVE_ROBOT_FWD_M_S * MAX_SPEED
					* (allianceFlip ? allianceOriented.getAsInt() : -1)
				)
			);
			drivetrain.setControl(brake);
			return;
		}

		double aSpeed = 0;

		if (tag != null) {
			double rpiTheta = tag.getPitch();

			aSpeed = Math.abs(rpiTheta)
				> VisionConstants.ROT_MARGIN_TO_REEF
				? 0.1 * Math.signum(rpiTheta) * MAX_ANGULAR_RATE : 0;

			Logger.recordOutput("Tag Z", tag.getZ());
			Logger.recordOutput("Tag X", tag.getX());
			Logger.recordOutput("Tag Angle", tag.getPitch());

			if (Utils.isSimulation()) {
				alignmentPose2d = getMapleSimDrivetrain()
					.getDriveSimulation().getSimulatedDriveTrainPose()
					.plus(new Transform2d(
						tag.getZ() + AutoConstants.REEF_X_TAG_OFFSET,
						-(tag.getX() + alignmentYOff),
						new Rotation2d()));
			} else {
				alignmentPose2d = drivetrain.getState().Pose
					.plus(new Transform2d(
						tag.getZ() + AutoConstants.REEF_X_TAG_OFFSET,
						-(tag.getX() + alignmentYOff),
						new Rotation2d())
					);
			}

			if (aSpeed == 0) {
				tagAlignedRotation = true;
			}

			Logger.recordOutput("rot speed", aSpeed);
		}

		if (alignmentPose2d != null) {
			Logger.recordOutput("ALignment Pose 2d", alignmentPose2d);

			double xDiff = (!Utils.isSimulation())
				? alignmentPose2d.getX() - drivetrain.getState().Pose.getX()
				: alignmentPose2d.getX() - getMapleSimDrivetrain().getDriveSimulation()
					.getSimulatedDriveTrainPose().getX();

			double yDiff = (!Utils.isSimulation())
				? alignmentPose2d.getY() - drivetrain.getState().Pose.getY()
				: alignmentPose2d.getY() - getMapleSimDrivetrain().getDriveSimulation()
					.getSimulatedDriveTrainPose().getY();

			double xSpeed;
			double ySpeed;

			if (xDiff > yDiff) {
				xSpeed = Math.abs(xDiff)
					> VisionConstants.X_MARGIN_TO_REEF
					? SwerveUtils.clamp(
						xDiff * VisionConstants.TRANSLATIONAL_ACCEL_CONSTANT,
						-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
						VisionConstants.MAX_SPEED_METERS_PER_SECOND
					) * MAX_SPEED : 0;

				ySpeed = Math.abs(yDiff)
					> VisionConstants.Y_MARGIN_TO_REEF
					? (xSpeed / xDiff * yDiff) : 0;
			} else {
				ySpeed = Math.abs(yDiff)
					> VisionConstants.Y_MARGIN_TO_REEF
					? SwerveUtils.clamp(
					yDiff * VisionConstants.TRANSLATIONAL_ACCEL_CONSTANT,
					-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
					VisionConstants.MAX_SPEED_METERS_PER_SECOND
				) * MAX_SPEED : 0;

				xSpeed = Math.abs(yDiff)
					> VisionConstants.X_MARGIN_TO_REEF
					? (ySpeed / yDiff * xDiff) : 0;
			}

			Logger.recordOutput("XSPEED", xSpeed);
			Logger.recordOutput("YSPEED", ySpeed);
			Logger.recordOutput("ASpeed", aSpeed);

			drivetrain.setControl(
				drive.withVelocityX(
					-xSpeed * ((allianceFlip) ? allianceOriented.getAsInt() : 1)
				)
				.withVelocityY(
					-ySpeed * ((allianceFlip) ? allianceOriented.getAsInt() : 1)
				)
				.withRotationalRate(-aSpeed)
			);

			System.out.println("REAHED SPEED SET TAG AL != NULL");

			// natural mk4 deadband
			tagPositionAligned =
				xSpeed == 0 && ySpeed == 0 && aSpeed == 0;
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

			private Timer passiveTimer;
			private boolean passiveForwardStageStarted = false;
			private boolean autoCompleted = false;

			AlignToReefTagCommand(int id) {
				tagID = id;
				tagPositionAligned = false; //likely redundant.
				passiveTimer = new Timer();
			}

			@Override
			public void initialize() {
				System.out.println("y: " + y);
				alignmentYOff = y;
				passiveTimer.reset();
			}

			@Override
			public void execute() {
				handleTagAlignment(null, id, false);

				if (tagPositionAligned && !passiveForwardStageStarted) {
					passiveTimer.start();
					passiveForwardStageStarted = true;
				}

				if (passiveForwardStageStarted) {
					if (passiveTimer.get() <= AutoConstants.PASSIVE_ROBOT_FWD_TIME_S) {
						drivetrain.setControl(
							driveRobotCentric.withVelocityX(
								DriveConstants.PASSIVE_ROBOT_FWD_M_S * MAX_SPEED
							)
						);
					} else {
						autoCompleted = true;
						drivetrain.setControl(brake);
					}
				}
			}

			@Override
			public boolean isFinished() {
				return autoCompleted;
			}

			@Override
			public void end(boolean interrupted) {
				System.out.println("ENDED");

				drivetrain.setControl(brake);
				tagPositionAligned = false;
				tagAlignedRotation = false;
				tagID = -1;
				alignmentPose2d = null;
				alignmentYOff = 0;
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

	/**
	 * Update the raspberry pi simulation state.
	 */
	public void updateRaspberryPi() {
		rpi.update(getMapleSimDrivetrain().getDriveSimulation().getSimulatedDriveTrainPose());
	}
}
