package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeter;

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
		new Rotation2d()
	);

	public static final int N_3 = 3;

	public static final Pose2d BLUE_2_STARTING_POS_M = new Pose2d(
		7.5856494,
		4.0468566,
		new Rotation2d()
	);

	public static final Pose2d BLUE_3_STARTING_POS_M = new Pose2d(
		7.5856494,
		1.5596578,
		new Rotation2d()
	);

	public static final Pose2d RED_1_STARTING_POS_M = new Pose2d(
		9.972452163696289,
		1.5596578,
		new Rotation2d(Math.PI)
	);
	public static final Pose2d RED_2_STARTING_POS_M = new Pose2d(
		9.972452163696289,
		4.0468566,
		new Rotation2d(Math.PI)
	);
	public static final Pose2d RED_3_STARTING_POS_M = new Pose2d(
		9.972452163696289,
		6.4390466,
		new Rotation2d(Math.PI)
	);

	// Estimated values for now, need to be calculated later
	public static final double MASS_WITH_BUMPER_LBS = 115;
	public static final double WIDTH_IN = 35.5;
	public static final double LENGTH_IN = 35.5;
	public static final double WHEEL_COF = 1.2;

	// mech pose logging constants
	public static final double ELEVATOR_WINCH_DIAMETER_METERS = 0.0463296;
	public static final double ELEVATOR_GEAR_RATIO = 10; //25.0;

	// The height of the funnel relative to the elevator stage 3 bottom.
	public static final double FUNNEL_HEIGHT_REL_ELEV_IN = 20;

	//vision related constants
	public static final String APRIL_TAG_FIELD_LAYOUT_JSON =
		"src/main/deploy/2025tagLayout.json";

	// Camera names, must match names configured on coprocessor
	public static final String REEF_CAMERA_NAME = "Reef CV Camera";
	public static final String STATION_CAMERA_NAME = "Station CV Camera";

	// Robot to camera transforms - not entirely accurate, but close enough for simulation
	// (Not used by Limelight, configure in web UI instead)
	public static final Transform3d ROBOT_TO_REEF_CAMERA =
		new Transform3d(0.25, 0, 0.4, new Rotation3d(0.0, 0.0, 0.0));
	public static final Transform3d ROBOT_TO_STATION_CAMERA =
		new Transform3d(-0.25, 0, 1.016, new Rotation3d(0.0, -0.0, Math.PI));

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
	public static final double FUNNEL_OUTTAKE_ROT_DEG = -30;
	public static final double FUNNEL_OUTTAKE_INIT_SPD_MPS = 1.1;
	public static final double N_1EN9 = 1e-9;

	public static final double STATION_BLUE_LEFT_FORWARD_X = 1.7287894487380981;
	public static final double STATION_BLUE_LEFT_FORWARD_Y = 8.0728206634521480;
	public static final double STATION_BLUE_LEFT_BACK_X =
			Meters.convertFrom(-61.00679561495781, Millimeter);
	public static final double STATION_BLUE_LEFT_BACK_Y = 8.0728206634521480;

	public static final double STATION_BLUE_RIGHT_FORWARD_X = 1.6924104690551758;
	public static final double STATION_BLUE_RIGHT_FORWARD_Y =
		Meters.convertFrom(-6.724564824253321, Millimeter);
	public static final double STATION_BLUE_RIGHT_BACK_X =
			Meters.convertFrom(-61.00679561495781, Millimeter);
	public static final double STATION_BLUE_RIGHT_BACK_Y = 1.2205365896224976;

	public static final double STATION_RED_LEFT_FORWARD_X = 15.837905883789062;
	public static final double STATION_RED_LEFT_FORWARD_Y =
			Meters.convertFrom(-24.513976648449898, Millimeter);
	public static final double STATION_RED_LEFT_BACK_X = 17.582408905029297;
	public static final double STATION_RED_LEFT_BACK_Y = 1.1875646114349365;

	public static final double STATION_RED_RIGHT_FORWARD_X = 15.783601760864258;
	public static final double STATION_RED_RIGHT_FORWARD_Y = 8.089945793151855;
	public static final double STATION_RED_RIGHT_BACK_X = 17.572349548339844;
	public static final double STATION_RED_RIGHT_BACK_Y = 6.808928489685059;
}
