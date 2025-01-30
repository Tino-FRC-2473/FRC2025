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
		// None of these are tuned at all

	public static final double ELEVATOR_DEADBAND = 0.1;

	public static final double ELEVATOR_PID_UPPER_THRESHOLD = 141.88; // DO NOT drive above this!
	public static final double ELEVATOR_PID_TARGET_L4 = 141.88;
	public static final double ELEVATOR_PID_TARGET_STATION = 66;
	public static final double ELEVATOR_PID_TARGET_GROUND = 0;

<<<<<<< HEAD
	public static final double ELEVATOR_MANUAL_SCALE = 0.7;
=======
	public static final double ELEVATOR_TARGET_MARGIN = 5;

	public static final double ELEVATOR_MANUAL_SCALE = 0.75;
>>>>>>> a840fea (remove pid from elevator, change state transition logic. TODO: rewrite auto commands)

	public static final double CLIMBER_PID_TARGET_LOW = 0;
	public static final double CLIMBER_PID_TARGET_EXTEND = 25;
	public static final double CLIMBER_PID_TARGET_CLIMB = 75;

	public static final double CLIMBER_COUNTS_PER_REV = 100;
	public static final double CLIMBER_PID_MARGIN_OF_ERROR = 0.05;

	// Motion Magic Constants
	public static final double ELEVATOR_MM_CONSTANT_G = 0.3;
		// Voltage required to overcome gravity
	public static final double ELEVATOR_MM_CONSTANT_S = 0.15;
		// Voltage required to overcome static friction

	// these two fields need to be retuned to account for 25:1 gearbox
	public static final double ELEVATOR_MM_CONSTANT_V = 0.1; // 0.1
		//Voltage for velocity of 1rps (0.1) retune
	public static final double ELEVATOR_MM_CONSTANT_A = 0.01; // 0.01
		//Voltage for acceleration of 1rps/s (0.01)

	public static final double ELEVATOR_MM_CONSTANT_P = 0.30;
		//Voltgae for Proportional error of 1 rot
	public static final double ELEVATOR_MM_CONSTANT_I = 0;
		//Voltage for Integrated error of 1 r*s
	public static final double ELEVATOR_MM_CONSTANT_D = 0;
		//Voltage for Derivative error of 1 rps

	public static final double ELEVATOR_CONFIG_CONSTANT_CV = 100; // Cruise Velo in rps
	public static final double ELEVATOR_CONFIG_CONSTANT_A = 200; // Max acceleration in rps/s
	public static final double ELEVATOR_CONFIG_CONSTANT_J = 2000; // Target jerk in rps/s/s

<<<<<<< HEAD
	public static final double CLIMB_POWER = 0.2;
=======
	public static final double ELEVATOR_POWER_DOWN = -0.5;
	public static final double ELEVATOR_POWER_UP = 0.7;
>>>>>>> a840fea (remove pid from elevator, change state transition logic. TODO: rewrite auto commands)

	// Other
	public static final int UPDATE_FREQUENCY_HZ = 100;
		// this is the lowest possible value since we refresh ourselves
		// changed from 4 --> 100
}
