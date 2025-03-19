package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;

// Values provided by Maple-Sim to reduce commonly found bugs while simulating.
public class SimConstants {
	public static final double MODULE_STEER_P = 70;
	public static final double MODULE_STEER_I = 0;
	public static final double MODULE_STEER_D = 4.5;
	public static final double MODULE_STEER_S = 0;
	public static final double MODULE_STEER_V = 1.91;
	public static final double MODULE_STEER_A = 0;

	public static final double DRIVE_FRICTION_VOLTS = 0.1;
	public static final double STEER_FRICTION_VOLTS = 0.05;

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
	public static final double ELEVATOR_SIM_VISUALIZATION_HEIGHT_MULTIPLIER = 2;
	public static final Translation3d CLIMBER_ZERO_POS = new Translation3d(
		0.05,
		0.305,
		0.412
		);
	public static final double CLIMBER_SIM_VISUALIZATION_CONSTANT = 0.0023;

	//done from the viewpoint of the drivers
	public static final int BLUE_STATION_LEFT_APRILTAG_ID = 13;
	public static final int BLUE_STATION_RIGHT_APRILTAG_ID = 12;
	public static final int RED_STATION_LEFT_APRILTAG_ID = 1;
	public static final int RED_STATION_RIGHT_APRILTAG_ID = 2;

	public static final double LOOP_PERIOD_MS = 0.020;
}
