package frc.robot.auto;

import frc.robot.constants.AutoConstants.AutoCommands;

public class AutoPaths {
	public static final Object[] B_PATH_1 = new Object[] {
		AutoCommands.FUNNEL_CLOSE_CMD, // should be redundant
		new Object[] {
			AutoCommands.ELEVATOR_STATION_CMD, "S2_R1"
		},
		new Object[] {
			AutoCommands.DRIVE_ROBOT_LEFT_RELATIVE_OFFSET_TIMED_CMD,
			AutoCommands.ELEVATOR_L4_CMD
		},
		AutoCommands.FUNNEL_OPEN_CMD,
		new Object[] {
			AutoCommands.ELEVATOR_STATION_CMD,
			"R1_StationL" //should check if it will return to the center before running w/ choreo
		},
		AutoCommands.FUNNEL_CLOSE_CMD,
		AutoCommands.B_ALIGN_REEF3_TAG_CMD,
		new Object[] {
			AutoCommands.DRIVE_ROBOT_LEFT_RELATIVE_OFFSET_TIMED_CMD,
			AutoCommands.ELEVATOR_L4_CMD
		},
		AutoCommands.FUNNEL_OPEN_CMD,
		AutoCommands.DRIVE_BRAKE_CMD
	};
}
