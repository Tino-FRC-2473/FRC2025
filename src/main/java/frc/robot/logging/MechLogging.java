package frc.robot.logging;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;

public final class MechLogging {
	public static final double WINCH_DIAMETER = 0.020501;

	private Pose3d primaryElevatorPose;
	private Pose3d secondaryElevatorPose;
	private Pose3d climberPose;

	private static MechLogging instance = new MechLogging();

	private MechLogging() {
		primaryElevatorPose = new Pose3d();
		secondaryElevatorPose = new Pose3d();
		climberPose = new Pose3d();
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
		double height = encoderSimPosition.in(Radians) * WINCH_DIAMETER;

		var pose = new Pose3d(
			new Translation3d(0, 0, height),
			Rotation3d.kZero
		);

		primaryElevatorPose = pose;
		secondaryElevatorPose = pose.div(2);
	}

	/**
	 * Updates the pose for the climber based on encoder position.
	 * @param encoderSimPosition the simulated location of the climber motor encoder.
	 */
	public void updatesClimberPose3d(Angle encoderSimPosition) {
		climberPose = new Pose3d(
			Translation3d.kZero,
			new Rotation3d(0, encoderSimPosition.in(Radians), 0)
		);
	}

	/**
	 * Get the primary elevator pose.
	 * @return the pose of the inner part of the elevator
	 */
	public Pose3d getPrimaryElevatorPose() {
		return primaryElevatorPose;
	}

	/**
	 * Get the secondary elevator pose.
	 * @return the pose of the inner-most part of the elevator
	 */
	public Pose3d getSecondaryElevatorPose() {
		return secondaryElevatorPose;
	}

	/**
	 * Get the pose of the climber ligament.
	 * @return pose of the rotating climber ligament.
	 */
	public Pose3d getClimberPose() {
		return climberPose;
	}
}
