package frc.robot.constants;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public final class Constants {
	// led constants
	public static final int LED_STRIP_BUFFER = 3; // TBD

	// funnel constants
	public static final double FUNNEL_CLOSED_POS_ROTS = 0.4;
	public static final double FUNNEL_OUTTAKE_POS_ROTS = 0.8;

	public static final double REEF_DISTANCE_THRESHOLD_MM = 100; // millimeters
	public static final double FUNNEL_INOUT_REAL_TIME_SECS = 0.5; // seconds
	public static final double FUNNEL_INOUT_SIM_TIME_SECS
		= FUNNEL_INOUT_REAL_TIME_SECS * 2; // seconds
	public static final double FUNNEL_INOUT_TIME_SECS =
		(Utils.isSimulation()) ? FUNNEL_INOUT_SIM_TIME_SECS : FUNNEL_INOUT_REAL_TIME_SECS;

	// PID Constants
	public static final double CLIMBER_INRANGE_VALUE = 0.5;

	//Elevator motion profile constants
	public static final double ELEVATOR_KG = 0.20;
	public static final double ELEVATOR_KS = 0.1;
	public static final double ELEVATOR_KV = 0.001;
	public static final double ELEVATOR_KA = 0.0;
	public static final double ELEVATOR_KP = 0.45;
	public static final double ELEVATOR_KI = 0.0;
	public static final double ELEVATOR_KD = 0.000; // may need to tune for going down

	public static final double ELEVATOR_CRUISE_VELO = 600;
	public static final double ELEVATOR_TARGET_ACCEL = 1800;
	public static final double ELEVATOR_EXPO_KV = 0.12;

	//Elevator Unit Conversion
	public static final double ELEVATOR_ROTS_TO_INCHES = 15 / (2 * Math.PI);

	// Encoder Position Constants
	public static final double ELEVATOR_JOYSTICK_INPUT_DEADBAND = 0.1;

	public static final Distance ELEVATOR_UPPER_THRESHOLD = Units.Inches.of(37.3);
	// DO NOT drive above this!
	public static final Distance ELEVATOR_TARGET_L4 = Units.Inches.of(37.3);
	public static final Distance ELEVATOR_TARGET_L3 = Units.Inches.of(25);
	public static final Distance ELEVATOR_TARGET_L2 = Units.Inches.of(15);
	public static final Distance ELEVATOR_TARGET_GROUND = Units.Inches.of(0);
	public static final Distance ELEVATOR_INRANGE_VALUE = Units.Inches.of(1);

	public static final double ELEVATOR_POWER = 0.8;
	public static final double ELEVATOR_REDUCED_POWER = 0.5;

	public static final double ELEVATOR_MANUAL_SCALE = 0.5;
	public static final double ELEVATOR_TARGET_MARGIN = 5;

	public static final double CLIMBER_PID_TARGET_LOW = 0;
	public static final double CLIMBER_PID_TARGET_EXTEND = 118;
	public static final double CLIMBER_PID_TARGET_CLIMB = 295;
	public static final double CLIMBER_ENCODER_RESET_POSITION = 310;

	public static final double CLIMBER_COUNTS_PER_REV = 427;
	public static final double CLIMBER_PID_MARGIN_OF_ERROR = 7;


	public static final double CLIMB_POWER = 0.5;
	public static final double CLIMB_REDUCED_POWER = 0.3;

	// Other
	public static final int UPDATE_FREQUENCY_HZ = 100;
		// this is the lowest possible value since we refresh ourselves
		// changed from 4 --> 100

	// Add units
	public static final double WINCH_DIAMETER_METERS = 0.0463296;
}
