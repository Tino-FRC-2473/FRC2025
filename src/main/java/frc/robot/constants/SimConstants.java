package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

// Values provided by Maple-Sim to reduce commonly found bugs while simulating.
public class SimConstants {
	public static final double MODULE_STEER_P = 70;
	public static final double MODULE_STEER_D = 4.5;

	public static final double DRIVE_FRICTION_VOLTS = 0.1;
	public static final double STEER_FRICTION_VOLTS = 0.15;

	public static final double STEER_INERTIA_KGMS2 = 0.05;

	public static final Pose2d BLUE_1_STARTING_POS_M = new Pose2d(
		7.5856494,
		6.4390466,
		new Rotation2d(Math.PI).minus(new Rotation2d(Math.PI/3))
	);

	public static final Pose2d BLUE_2_STARTING_POS_M = new Pose2d(
		7.5856494,
		4.0468566,
		new Rotation2d(Math.PI)
	);

	public static final Pose2d BLUE_3_STARTING_POS_M = new Pose2d(
		7.5856494,
		1.5596578,
		new Rotation2d(Math.PI).plus(new Rotation2d(Math.PI/3))
	);

	public static final Pose2d RED_1_STARTING_POS_M = new Pose2d(
		9.972452163696289,
		1.5596578,
		new Rotation2d().plus(new Rotation2d(Math.PI/3))
	);
	public static final Pose2d RED_2_STARTING_POS_M = new Pose2d(
		9.972452163696289,
		4.0468566,
		new Rotation2d()
	);
	public static final Pose2d RED_3_STARTING_POS_M = new Pose2d(
		9.972452163696289,
		6.4390466,
		new Rotation2d().minus(new Rotation2d(Math.PI/3))
	);

	// Estimated values for now, need to be calculated later
	public static final double MASS_WITH_BUMPER_LBS = 115;
	public static final double MOI = 6.99597;
	public static final double WIDTH_IN = 35.5;
	public static final double LENGTH_IN = 35.5;
	public static final double WHEEL_COF = 1.2;

	public static final Translation2d FL_TRANSLATION =
		new Translation2d(
			10.75,
			-10.75
		);

	public static final Translation2d FR_TRANSLATION =
		new Translation2d(
			10.75,
			10.75
		);

	public static final Translation2d BL_TRANSLATION =
		new Translation2d(
			-10.75,
			-10.75
		);

	public static final Translation2d BR_TRANSLATION =
		new Translation2d(
			-10.75,
			10.75
		);

	// mech pose logging constants
	public static final double ELEVATOR_WINCH_DIAMETER_METERS = 0.0463296;
	public static final double ELEVATOR_GEAR_RATIO = 10; //25.0;

	// Camera names, must match names configured on coprocessor
	public static final String REEF_CAMERA_NAME = "Reef CV Camera";
	public static final String STATION_CAMERA_NAME = "Station CV Camera";

	public static final double STATION_CAMERA_RAD = Math.toRadians(19);

	// Robot to camera transforms
	// (Not used by Limelight, configure in web UI instead)
	public static final Transform3d ROBOT_TO_REEF_CAMERA =
		new Transform3d(Units.inchesToMeters(7.129), -Units.inchesToMeters(4.306),
		Units.inchesToMeters(14.56), new Rotation3d(0.0, 0.0, 0.0));
	public static final Transform3d ROBOT_TO_STATION_CAMERA =
		new Transform3d(-Units.inchesToMeters(8.875), -Units.inchesToMeters(9.5),
		Units.inchesToMeters(37.596), new Rotation3d(0.0, -STATION_CAMERA_RAD, Math.PI));

	// Basic filtering thresholds
	public static final double MAX_AMBIGUITY = 0.3;
	public static final double MAX_Z_ERROR = 0.75;

	// Standard deviation baselines, for 1 meter distance and 1 tag
	// (Adjusted automatically based on distance and # of tags)
	public static final double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
	public static final double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

	// Standard deviation multipliers for each camera
	// (Adjust to trust some cameras more than others)
	public static final double[] CAMERA_STD_DEV_MULTIPLIERS =
		new double[] {
			1.0, // Camera 0
			1.0 // Camera 1
		};

	// Multipliers to apply for MegaTag 2 observations
	public static final double LINEAR_STD_MEGATAG_2_FACTOR = 0.5; // More stable than full 3D solve
	public static final double ANGULAR_STD_MEGATAG_2_FACTOR =
		Double.POSITIVE_INFINITY; // No rotation data available

	// public static final double CAM_DISTANCE_READ = 2.5;
}
