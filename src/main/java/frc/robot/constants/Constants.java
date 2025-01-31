package frc.robot.constants;

public final class Constants {
	// led constants
	public static final int LED_STRIP_BUFFER = 3; // TBD

	// funnel constants

	public static final double FUNNEL_CLOSED_POS_ROTS = 0;
	public static final double FUNNEL_OUTTAKE_POS_ROTS = 0.4;

	public static final double REEF_DISTANCE_THRESHOLD_MM = 100; // millimeters
	public static final double FUNNEL_CLOSE_TIME_SECS = 0.5; // seconds

	// PID Constants
	public static final double CLIMBER_INRANGE_VALUE = 0.5;

	// Encoder Position Constants
	public static final double ELEVATOR_DEADBAND = 0.1;

	public static final double ELEVATOR_UPPER_THRESHOLD = 146; // DO NOT drive above this!
	public static final double ELEVATOR_TARGET_L4 = 146;
	public static final double ELEVATOR_TARGET_L3 = 78;
	public static final double ELEVATOR_TARGET_L2 = 30;
	public static final double ELEVATOR_TARGET_GROUND = 0;
	public static final double ELEVATOR_SPEED_REDUCTION_THRESHOLD_SIZE = 15;

	public static final double ELEVATOR_POWER = 0.8;
	public static final double ELEVATOR_REDUCED_POWER = 0.5;

	public static final double ELEVATOR_MANUAL_SCALE = 0.8;
	public static final double ELEVATOR_TARGET_MARGIN = 5;

	public static final double CLIMBER_PID_TARGET_LOW = 0;
	public static final double CLIMBER_PID_TARGET_EXTEND = 25;
	public static final double CLIMBER_PID_TARGET_CLIMB = 75;

	public static final double CLIMBER_COUNTS_PER_REV = 100;
	public static final double CLIMBER_PID_MARGIN_OF_ERROR = 0.05;


	public static final double CLIMB_POWER = 0.2;

	// Other
	public static final int UPDATE_FREQUENCY_HZ = 100;
		// this is the lowest possible value since we refresh ourselves
		// changed from 4 --> 100

	// Add units
	public static final double WINCH_DIAMETER_METERS = 0.0463296;
}
