package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/** These will be constants associated with each auto path in AutoRoutines. */
public class AutoConstants {

	/* -- ALL GENERAL CONSTANTS -- */
	public static final double DEG_180 = 180;
	public static final double DEG_360 = 360;

	/* -- ALL SOURCE SPECIFIC CONSTANTS -- */
	public static final double SOURCE_X_OFFSET = (Units.inchesToMeters(35.5 / 2 - 6));
	public static final double SOURCE_Y_OFFSET = 0;

	public static final int BLUE_L_STATION_ID = 13;
	public static final int BLUE_R_STATION_ID = 12;
	public static final int RED_L_STATION_ID = 1;
	public static final int RED_R_STATION_ID = 2;


	public static final double REEF_X_TAG_OFFSET = Units.inchesToMeters(35.5 / 2 - 6 - 4);
	public static final double REEF_Y_L_TAG_OFFSET = -Units.inchesToMeters(12) / 2;
	public static final double REEF_Y_R_TAG_OFFSET = Units.inchesToMeters(12) / 2;
	public static final double STATION_Y_L_TAG_OFFSET = Units.inchesToMeters(18);
	public static final double STATION_Y_R_TAG_OFFSET = -Units.inchesToMeters(18);

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

	public static final double ALIGN_DRIVE_P = 1.7; //2.2; //4.0;
	public static final double ALIGN_THETA_P = 1.5; //1.3; //2.0;
	public static final double ALIGN_MAX_T_SPEED = 0.0;
	public static final double ALIGN_MAX_T_ACCEL = 0.0;
	public static final double ALIGN_MAX_R_SPEED = Math.PI * 0.0;
	public static final double ALIGN_MAX_R_ACCEL = Math.PI * 0.0;
	public static final double FF_MIN_RADIUS = 0.2;
	public static final double FF_MAX_RADIUS = 0.8;
	public static final double CHOREO_PID_TOLLERANCE = 1e-3;

	public static final int DRIVE_CURRENT_LIMIT_FRAMES = 2;


	/* -- ALL COMMAND NAME CONSTANTS -- */

	public enum AutoCommands {
		/* Red Align Reef Tag Commands */
		R_ALIGN_REEF1_L_TAG_CMD,
		R_ALIGN_REEF1_R_TAG_CMD,
		R_ALIGN_REEF2_L_TAG_CMD,
		R_ALIGN_REEF2_R_TAG_CMD,
		R_ALIGN_REEF3_L_TAG_CMD,
		R_ALIGN_REEF3_R_TAG_CMD,
		R_ALIGN_REEF4_L_TAG_CMD,
		R_ALIGN_REEF4_R_TAG_CMD,
		R_ALIGN_REEF5_L_TAG_CMD,
		R_ALIGN_REEF5_R_TAG_CMD,
		R_ALIGN_REEF6_L_TAG_CMD,
		R_ALIGN_REEF6_R_TAG_CMD,
		/* Blue Align Reef Tag Commands */
		B_ALIGN_REEF1_L_TAG_CMD,
		B_ALIGN_REEF1_R_TAG_CMD,
		B_ALIGN_REEF2_L_TAG_CMD,
		B_ALIGN_REEF2_R_TAG_CMD,
		B_ALIGN_REEF3_L_TAG_CMD,
		B_ALIGN_REEF3_R_TAG_CMD,
		B_ALIGN_REEF4_L_TAG_CMD,
		B_ALIGN_REEF4_R_TAG_CMD,
		B_ALIGN_REEF5_L_TAG_CMD,
		B_ALIGN_REEF5_R_TAG_CMD,
		B_ALIGN_REEF6_L_TAG_CMD,
		B_ALIGN_REEF6_R_TAG_CMD,
		/* Align Station Tag Commands */
		R_ALIGN_STATION_L_TAG_CMD,
		R_ALIGN_STATION_R_TAG_CMD,
		B_ALIGN_STATION_L_TAG_CMD,
		B_ALIGN_STATION_R_TAG_CMD,
		/* Drive Peripheral Commands */
		DRIVE_BRAKE_CMD,
		/* Elevator Commands */
		ELEVATOR_GROUND_CMD,
		ELEVATOR_L2_CMD,
		ELEVATOR_L3_CMD,
		ELEVATOR_L4_CMD,
		ELEVATOR_WAIT,
		/* Funnel Commands */
		INTAKE_CORAL_CMD,
		OUTTAKE_CORAL_CMD
	}

}
