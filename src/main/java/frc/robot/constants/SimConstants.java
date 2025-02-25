package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;

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

	// mech pose logging constants
	public static final double ELEVATOR_WINCH_DIAMETER_METERS = 0.0463296;
	public static final double ELEVATOR_GEAR_RATIO = 10; //25.0;
	public static final Translation3d CLIMBER_ZERO_POS = new Translation3d(
		0.05,
		0.305,
		0.412
		);
	public static final double CLIMBER_GEAR_RATIO = 50;
}
