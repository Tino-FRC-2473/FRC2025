package frc.robot.logging;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;

public class MechLogging {

	public static final double WINCH_DIAMETER = 0.020501;

	private Pose3d elevatorSegment1Pose3d;
	private Pose3d elevatorSegment2Pose3d;
	private Pose3d climberPose3d;

	/* What to publish over networktables for telemetry */
	private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

	private final NetworkTable elevatorStateTable = inst.getTable("ElevatorState");
	private final NetworkTable climberStateTable = inst.getTable("ClimberState");

	/* Gets the primary pose of the elevator */
	private final StructPublisher<Pose3d>
		elevatorSegment1PosePublisher = elevatorStateTable.getStructTopic("Pose1", Pose3d.struct)
		.publish();

	/* Gets the secondary pose of the elevator */
	private final StructPublisher<Pose3d>
		elevatorSegment2PosePublisher = elevatorStateTable.getStructTopic("Pose2", Pose3d.struct)
		.publish();

	/* Gets the pose of the climber */
	private final StructPublisher<Pose3d>
		climberPosePublisher = climberStateTable.getStructTopic("Pose", Pose3d.struct)
		.publish();


	/**
	 * Generates the pose for the elevator based on encoder position.
	 * @param encoderSimPosition the simulated location of the elevator motor encoder.
	 */
	public void updateElevatorPose3d(Angle encoderSimPosition) {
		double height = encoderSimPosition.in(Radians) * WINCH_DIAMETER;
		Pose3d pose = new Pose3d(
			new Translation3d(0, 0, height),
			Rotation3d.kZero
		);
		elevatorSegment1Pose3d = pose;
		elevatorSegment2Pose3d = pose.div(2);

		elevatorSegment1PosePublisher.set(elevatorSegment1Pose3d);
		elevatorSegment2PosePublisher.set(elevatorSegment2Pose3d);
	}

	/**
	 * Updates the pose for the climber based on encoder position.
	 * @param encoderSimPosition the simulated location of the climber motor encoder.
	 */
	public void updatesClimberPose3d(Angle encoderSimPosition) {
		Pose3d pose = new Pose3d(
			Translation3d.kZero,
			new Rotation3d(0, encoderSimPosition.in(Radians), 0)
		);
		climberPose3d = pose;

		climberPosePublisher.set(climberPose3d);
	}
}
