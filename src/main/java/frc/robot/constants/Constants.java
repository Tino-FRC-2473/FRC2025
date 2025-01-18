package frc.robot.constants;

public final class Constants {

	//funnel constants

	public static final double FUNNEL_CLOSED_POS = 1;

	public static final double FUNNEL_OUTTAKE_POS = 0;

	// PID Constants
	public static final double INRANGE_VALUE = 0.5;

	// Encoder Position Constants
		// None of these are tuned at all

	public static final double ELEVATOR_PID_TARGET_L4 = 200;
	public static final double ELEVATOR_PID_TARGET_GROUND = 50;
	public static final double ELEVATOR_PID_TARGET_STATION = 150;

	public static final double CLIMBER_PID_TARGET_LOW = 0;
	public static final double CLIMBER_PID_TARGET_EXTEND = 25;
	public static final double CLIMBER_PID_TARGET_CLIMB = 75;

	public static final double CLIMBER_EXTERNAL_GEAR_RATIO = 100;

	// Motion Magic Constants
	public static final double ELEVATOR_MM_CONSTANT_G = 0.17;
		// Voltage required to overcome gravity
	public static final double ELEVATOR_MM_CONSTANT_S = 0.1; //0.10
		//Voltage required to overcome static friction (0.15)
	public static final double ELEVATOR_MM_CONSTANT_V = 0.1; // 0.1
		//Voltage for velocity of 1rps (0.1) retune
	public static final double ELEVATOR_MM_CONSTANT_A = 0.01; // 0.01
		//Voltage for acceleration of 1rps/s (0.01)
	public static final double ELEVATOR_MM_CONSTANT_P = 0.3; // 0.9
		//Voltgae for Proportional error of 1 rot(0.7)
	public static final double ELEVATOR_MM_CONSTANT_I = 0.005;
		//Voltage for Integrated error of 1 r*s
	public static final double ELEVATOR_MM_CONSTANT_D = 0.0;
		//Voltage for Derivative error of 1 rps

	public static final double CLIMBER_MM_CONSTANT_G = 0.17;
		// Voltage required to overcome gravity
	public static final double CLIMBER_MM_CONSTANT_S = 0.1; //0.10
		//Voltage required to overcome static friction (0.15)
	public static final double CLIMBER_MM_CONSTANT_V = 0.1; // 0.1
		//Voltage for velocity of 1rps (0.1) retune
	public static final double CLIMBER_MM_CONSTANT_A = 0.01; // 0.01
		//Voltage for acceleration of 1rps/s (0.01)
	public static final double CLIMBER_MM_CONSTANT_P = 0.3; // 0.9
		//Voltgae for Proportional error of 1 rot(0.7)
	public static final double CLIMBER_MM_CONSTANT_I = 0.005;
		//Voltage for Integrated error of 1 r*s
	public static final double CLIMBER_MM_CONSTANT_D = 0.0;
		//Voltage for Derivative error of 1 rps


	public static final double ELEVATOR_CONFIG_CONSTANT_CV = 14; // Cruise Velo in rps (10)
	public static final double ELEVATOR_CONFIG_CONSTANT_A = 80; // Max acceleration in rps/s (80)
	public static final double ELEVATOR_CONFIG_CONSTANT_J = 110; // Target jerk in rps/s/s (110)

	// to be tuned for climber
	public static final double CLIMBER_CONFIG_CONSTANT_CV = 14; // Cruise Velo in rps (10)
	public static final double CLIMBER_CONFIG_CONSTANT_A = 80; // Max acceleration in rps/s (80)
	public static final double CLIMBER_CONFIG_CONSTANT_J = 110; // Target jerk in rps/s/s (110)

	// Other
	public static final int UPDATE_FREQUENCY_HZ = 100;
		// this is the lowest possible value since we refresh ourselves
		// changed from 4 --> 100
}
