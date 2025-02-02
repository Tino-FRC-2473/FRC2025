package frc.robot.logging;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.Constants;

public final class MechLogging {
	private Pose3d elevatorStage1;
	private Pose3d elevatorStage2;
	private Pose3d elevatorStage3;

	private Pose3d climberPost;
	private Pose3d climberPivot;

	private Pose3d drivePose;

	private static MechLogging instance = new MechLogging();

	private MechLogging() {
		elevatorStage1 = new Pose3d();
		elevatorStage2 = new Pose3d();
		elevatorStage3 = new Pose3d();
		climberPost = new Pose3d();
		drivePose = new Pose3d();
	}

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
		double totalHeight = encoderSimPosition.in(Radians) * Constants.WINCH_DIAMETER_METERS / 2;
		System.out.println("Total Height " + totalHeight);

		// Stage 1 (bottom stage) moves 1/6 of the total movement
		elevatorStage1 = new Pose3d(
			drivePose.getTranslation().plus(new Translation3d(0, 0, totalHeight / 6)),
			drivePose.getRotation()
		);

		// Stage 2 (middle stage) moves 1/3 of the total movement
		elevatorStage2 = new Pose3d(
			drivePose.getTranslation().plus(new Translation3d(0, 0, totalHeight / 3)),
			drivePose.getRotation()
		);

		// Stage 3 (top stage) moves 1/2 of the total movement
		elevatorStage3 = new Pose3d(
			drivePose.getTranslation().plus(new Translation3d(0, 0, totalHeight / 2)),
			drivePose.getRotation()
		);
	}

	/**
	 * Updates the pose for the climber based on encoder position.
	 * @param encoderSimPosition the simulated location of the climber motor encoder.
	 */
	public void updatesClimberPose3d(Angle encoderSimPosition) {
		climberPost = drivePose;

		climberPivot = new Pose3d(
			Translation3d.kZero,
			new Rotation3d(0, encoderSimPosition.in(Radians), 0)
		);
	}

	/**
	 * Get the outer-most elevator pose.
	 * @return outer-most elevator pose
	 */
	public Pose3d getElevatorStage1Pose() {
		return drivePose;
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
	public Pose3d getClimberPost() {
		return climberPost;
	}

	/**
	 * Get the pose of the climber pivot.
	 * @return pose of the climber pivot
	 */
	public Pose3d getClimberPivot() {
		return climberPivot;
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
		};
	}

	/**
	 * Sets the drive pose data, used to determine the components' absolute location.
	 * @param pose the 2d pose of the robot
	 */
	public void setDrivePoseData(Pose2d pose) {
		this.drivePose = new Pose3d(pose);
	}
}
