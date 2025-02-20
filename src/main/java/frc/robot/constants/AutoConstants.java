package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/** These will be constants associated with each auto path in AutoRoutines. */
public class AutoConstants {

	/* -- ALL GENERAL CONSTANTS -- */
	public static final double DEG_180 = 180;
	public static final double DEG_360 = 360;

	/* -- ALL SOURCE SPECIFIC CONSTANTS -- */
	public static final double SOURCE_X_OFFSET = 0;
	public static final double SOURCE_Y_OFFSET = 0;

	public static final int BLUE_L_STATION_ID = 12;
	public static final int BLUE_R_STATION_ID = 13;
	public static final int RED_L_STATION_ID = 2;
	public static final int RED_R_STATION_ID = 1;

	/* -- ALL REEF SPECIFIC CONSTANTS -- */
	public static final double REEF_X_L_TAG_OFFSET = -0.5;
	public static final double REEF_X_R_TAG_OFFSET = 0.5;
	public static final double REEF_Y_L_TAG_OFFSET = 0;
	public static final double REEF_Y_R_TAG_OFFSET = 0;

	public static final int R_REEF_1_TAG_ID = 10;
	public static final int R_REEF_2_TAG_ID = 11;
	public static final int R_REEF_3_TAG_ID = 6;
	public static final int R_REEF_4_TAG_ID = 7;
	public static final int R_REEF_5_TAG_ID = 8;
	public static final int R_REEF_6_TAG_ID = 9;

	public static final int B_REEF_1_TAG_ID = 21;
	public static final int B_REEF_2_TAG_ID = 20;
	public static final int B_REEF_3_TAG_ID = 19;
	public static final int B_REEF_4_TAG_ID = 18;
	public static final int B_REEF_5_TAG_ID = 17;
	public static final int B_REEF_6_TAG_ID = 22;

	public static final double TIME_DRIVING_OFFSET = 0.2;

	// 13 inches is the distance between pipes, center to center
	public static final double ABS_REEF_OFFSET_Y_DISTANCE_METERS = Units.inchesToMeters(13) / 2;
	public static final double ABS_REEF_OFFSET_X_DISTANCE_METERS = 0;

	//Calculated final offset constants
	public static final double REEF_OFFSET_X_AUTO_SPEED_M_S
		= ABS_REEF_OFFSET_X_DISTANCE_METERS / TIME_DRIVING_OFFSET;
	public static final double REEF_OFFSET_Y_AUTO_SPEED_M_S
		= ABS_REEF_OFFSET_Y_DISTANCE_METERS / TIME_DRIVING_OFFSET;


	/* -- ALL COMMAND NAME CONSTANTS -- */

	public enum AutoCommands {
		/* Red Align Reef Tag Commands */
		R_ALIGN_REEF2_TAG_CMD,
		R_ALIGN_REEF3_TAG_CMD,
		R_ALIGN_REEF5_TAG_CMD,
		R_ALIGN_REEF6_TAG_CMD,
		/* Blue Align Reef Tag Commands */
		B_ALIGN_REEF2_TAG_CMD,
		B_ALIGN_REEF3_TAG_CMD,
		B_ALIGN_REEF5_TAG_CMD,
		B_ALIGN_REEF6_TAG_CMD,
		/* Align Station Tag Commands */
		R_ALIGN_STATION_L_TAG_CMD,
		R_ALIGN_STATION_R_TAG_CMD,
		B_ALIGN_STATION_L_TAG_CMD,
		B_ALIGN_STATION_R_TAG_CMD,
		/* Drive Peripheral Commands */
		DRIVE_BRAKE_CMD,
		DRIVE_ROBOT_LEFT_RELATIVE_OFFSET_TIMED_CMD,
		DRIVE_ROBOT_RIGHT_RELATIVE_OFFSET_TIMED_CMD,
		/* Elevator Commands */
		ELEVATOR_GROUND_CMD,
		ELEVATOR_L2_CMD,
		ELEVATOR_L3_CMD,
		ELEVATOR_L4_CMD,
		WAIT,
		/* Funnel Commands */
		INTAKE_CORAL_CMD,
		OUTTAKE_CORAL_CMD
	}
}
