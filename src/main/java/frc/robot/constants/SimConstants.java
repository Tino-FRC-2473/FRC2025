package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

// Values provided by Maple-Sim to reduce commonly found bugs while simulating.
public class SimConstants {
	public static final double MODULE_STEER_P = 70;
	public static final double MODULE_STEER_D = 4.5;

	public static final double DRIVE_FRICTION_VOLTS = 0.1;
	public static final double STEER_FRICTION_VOLTS = 0.15;

	public static final double STEER_INERTIA_KGMS2 = 0.05;

	public static final double STARTING_POS_X_FT = 5;
	public static final double STARTING_POS_Y_FT = 5;

	// Estimated values for now, need to be calculated later
	public static final double MASS_WITH_BUMPER_LBS = 115;
	public static final double WIDTH_IN = 35.5;
	public static final double LENGTH_IN = 35.5;
	public static final double WHEEL_COF = 1.2;

	public enum CoralStation {
		BLUE_LEFT,
		BLUE_RIGHT,
		RED_RIGHT,
		RED_LEFT
	}

	// mech pose logging constants
	public static final double ELEVATOR_WINCH_DIAMETER_METERS = 0.0463296;
	public static final double ELEVATOR_GEAR_RATIO = 3; //25.0;
	public static final Translation3d CLIMBER_ZERO_POS = new Translation3d(
		0.05,
		0.305,
		0.412
		);
	public static final double CLIMBER_GEAR_RATIO = 50;

	//done from the viewpoint of the drivers
	public static final int BLUE_STATION_LEFT_APRILTAG_ID = 13;
	public static final int BLUE_STATION_RIGHT_APRILTAG_ID = 12;
	public static final int RED_STATION_LEFT_APRILTAG_ID = 1;
	public static final int RED_STATION_RIGHT_APRILTAG_ID = 2;

	//conversion from cm to meters
	public static final double CORAL_STATION_WIDTH_METERS = Units.inchesToMeters(76);

	//max distance coral can be intaked away from station, in meters
	public static final double MAX_DISTANCE_FROM_STATION = 0.02;

	//Difference between heights of the stage 3 elevator and outtake
	public static final double OUTTAKE_OFFSET = Units.inchesToMeters(18);
}
