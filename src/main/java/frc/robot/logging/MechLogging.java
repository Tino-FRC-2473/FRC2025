package frc.robot.logging;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.SimConstants;

public final class MechLogging {
	private Pose3d elevatorStage1;
	private Pose3d elevatorStage2;
	private Pose3d elevatorStage3;
	private Pose3d climberPose;

	private static MechLogging instance = new MechLogging();

	private boolean doesSimRobotHaveCoral = true; // Preloaded 1

	private Pose2d drivetrainPose;
	private ChassisSpeeds drivetrainChassisSpeeds;
	private Rotation2d driveRotation;

	private MechLogging() {
		elevatorStage1 = new Pose3d();
		elevatorStage2 = new Pose3d();
		elevatorStage3 = new Pose3d();
		climberPose = new Pose3d();
	}

	public static final double STAGE_2_RATIO = 1.0 / 3.0;
	public static final double STAGE_3_RATIO = 2.0 / 3.0;

	/**
	 * Get the instance of the singleton class.
	 * @return the instance
	 */
	public static MechLogging getInstance() {
		return instance;
	}

	/**
	 * Generates the pose for the elevator based on encoder position.
	 * @param encoderSimPosition the simulated location of the elevator motor encoder.
	 */
	public void updateElevatorPose3d(Angle encoderSimPosition) {
		double totalHeight =
			encoderSimPosition
				.in(Radians)
			* SimConstants.ELEVATOR_SIM_VISUALIZATION_HEIGHT_MULTIPLIER;

		// Stage 1 (bottom stage) moves 1/6 of the total movement
		elevatorStage1 = new Pose3d(
			Pose3d.kZero
				.getTranslation().plus(new Translation3d(0, 0, 0)),
			Rotation3d.kZero
		);

		// Stage 2 (middle stage) moves 1/3 of the total movement
		elevatorStage2 = new Pose3d(
			Pose3d.kZero
				.getTranslation().plus(new Translation3d(0, 0, totalHeight * STAGE_2_RATIO)),
			Rotation3d.kZero
		);

		// Stage 3 (top stage) moves 1/2 of the total movement
		elevatorStage3 = new Pose3d(
			Pose3d.kZero
				.getTranslation()
				.plus(new Translation3d(0, 0, totalHeight * STAGE_3_RATIO)),
			Rotation3d.kZero
		);
	}

	/**
	 * Updates the pose for the climber based on encoder position.
	 * @param encoderSimPosition the simulated location of the climber motor encoder.
	 */
	public void updatesClimberPose3d(Angle encoderSimPosition) {
		climberPose = new Pose3d(
			SimConstants.CLIMBER_ZERO_POS,
			new Rotation3d(
				encoderSimPosition
				.times(SimConstants.CLIMBER_SIM_VISUALIZATION_CONSTANT).in(Radians),
				0,
				0
			)
		);
	}

	/**
	 * Get the primary elevator pose.
	 * @return the pose of the inner part of the elevator
	 */
	public Pose3d getElevatorStage1() {
		return elevatorStage1;
	}

	/**
	 * Get the secondary elevator pose.
	 * @return the pose of the inner-most part of the elevator
	 */
	public Pose3d getElevatorStage2() {
		return elevatorStage2;
	}

	/**
	 * Get the elevator stage 3 position.
	 * @return the elevator stage 3 position
	 */
	public Pose3d getElevatorStage3() {
		return elevatorStage3;
	}

	/**
	 * Get the pose of the climber ligament.
	 * @return pose of the rotating climber ligament.
	 */
	public Pose3d getClimberPose() {
		return climberPose;
	}

	/**
	 * Get the pose of the intake.
	 * @return the pose of the intake
	 */
	public Pose3d getIntakePose() {
		return elevatorStage1;
	}

	/**
	 * Get the pose of the outtake.
	 * @return the pose of the outtake
	 */
	public Pose3d getOuttakePose() {
		return elevatorStage3;
	}

	/**
	 * Getter for the array of poses we want to simulate.
	 * @return the array of poses to display in advantageScope
	 */
	public Pose3d[] getRobotPoses() {
		return new Pose3d[]{
			getElevatorStage1(),
			getElevatorStage2(),
			getElevatorStage3(),
			getClimberPose(),
			getIntakePose(),
			getOuttakePose()
		};
	}

	// /**
	//  * Gets if the sim robot has a coral loaded in the robot.
	//  * @return if the sim robot has a coral loaded in the robot
	//  */
	// public boolean doesSimRobotHaveCoral() {
	// 	return doesSimRobotHaveCoral;
	// }

	/**
	 * Updates the drivetrain values.
	 * @param pose the pose of the drivetrain
	 * @param speeds the chassis speeds of the drivetrain
	 * @param rotation the rotation of the drivetrain
	 */
	public void updateDrivetrainValues(Pose2d pose, ChassisSpeeds speeds, Rotation2d rotation) {
		drivetrainPose = pose;
		drivetrainChassisSpeeds = speeds;
		driveRotation = rotation;
	}
}
