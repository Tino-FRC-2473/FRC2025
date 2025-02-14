package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeter;

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
	public static final double ELEVATOR_GEAR_RATIO = 10; // 25.0;

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