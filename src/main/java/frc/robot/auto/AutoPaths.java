package frc.robot.auto;

import java.lang.reflect.Field;
import java.util.HashMap;

import frc.robot.constants.AutoConstants.AutoCommands;

public class AutoPaths {
	public static final Object[] B_PATH_1 = new Object[] {
		new Object[] {
			AutoCommands.ELEVATOR_L2_CMD, "S2_R1"
		},
		new Object[] {
			AutoCommands.DRIVE_ROBOT_LEFT_RELATIVE_OFFSET_TIMED_CMD,
			AutoCommands.ELEVATOR_L4_CMD
		},
		AutoCommands.OUTTAKE_CORAL_CMD,
		new Object[] {
			AutoCommands.ELEVATOR_L2_CMD,
			"R1_StationL" //should check if it will return to the center before running w/ choreo
		},
		AutoCommands.INTAKE_CORAL_CMD,
		AutoCommands.B_ALIGN_REEF3_L_TAG_CMD,
		new Object[] {
			AutoCommands.DRIVE_ROBOT_LEFT_RELATIVE_OFFSET_TIMED_CMD,
			AutoCommands.ELEVATOR_L4_CMD
		},
		AutoCommands.OUTTAKE_CORAL_CMD
	};

	public static final Object[] B_PATH_2 = new Object[] {
		"S1_R2",
		"R2_StationL",
		"StationL_R4",
		AutoCommands.DRIVE_ROBOT_LEFT_RELATIVE_OFFSET_TIMED_CMD
	};

	public static final Object[] B_PATH_3 = new Object[] {
		"S1_R2",
		AutoCommands.ELEVATOR_L4_CMD,
		AutoCommands.ELEVATOR_L2_CMD
	};

	public static final Object[] B_AT_ALIGN_S1 = new Object[] {
		"S1_R2_H",
		new Object[] {AutoCommands.B_ALIGN_REEF2_L_TAG_CMD, AutoCommands.ELEVATOR_L3_CMD},
		AutoCommands.ELEVATOR_L4_CMD,
		AutoCommands.OUTTAKE_CORAL_CMD,
		new Object[] {"R2_StationL", AutoCommands.ELEVATOR_GROUND_CMD},
		AutoCommands.INTAKE_CORAL_CMD,
		new Object[] {AutoCommands.B_ALIGN_REEF3_R_TAG_CMD, AutoCommands.ELEVATOR_L3_CMD},
		AutoCommands.ELEVATOR_L4_CMD,
		AutoCommands.OUTTAKE_CORAL_CMD
	};

	public static final Object[] B_S2 = new Object[] {
		new Object[] {"S2_R2", AutoCommands.ELEVATOR_L3_CMD},
		AutoCommands.ELEVATOR_L4_CMD,
		AutoCommands.OUTTAKE_CORAL_CMD,
		AutoCommands.ELEVATOR_GROUND_CMD,
	};

	public static final Object[] B_AT_ALIGN_S3 = new Object[] {
		"S3_R6 (H)",
		new Object[] {AutoCommands.B_ALIGN_REEF6_L_TAG_CMD, AutoCommands.ELEVATOR_L3_CMD},
		AutoCommands.ELEVATOR_L4_CMD,
		AutoCommands.OUTTAKE_CORAL_CMD,
		new Object[] {"R6_StationR", AutoCommands.ELEVATOR_GROUND_CMD},
		AutoCommands.INTAKE_CORAL_CMD,
		"StationR_R5 (H)",
		new Object[] {AutoCommands.B_ALIGN_REEF5_L_TAG_CMD, AutoCommands.ELEVATOR_L3_CMD},
		AutoCommands.ELEVATOR_L4_CMD,
		AutoCommands.OUTTAKE_CORAL_CMD,
		AutoCommands.ELEVATOR_GROUND_CMD
	};

	public static final Object[] R_AT_ALIGN_S3 = new Object[] {
		"S1_R2 (H)",
		new Object[] {AutoCommands.R_ALIGN_REEF2_L_TAG_CMD, AutoCommands.ELEVATOR_L3_CMD},
	};

	/**
	 * Get all autos declared in the file.
	 * @return hashmap of auto name and autos.
	 */
	public HashMap<String, Object[]> getAllAutos() {
		HashMap<String, Object[]> allObjArrays = new HashMap<String, Object[]>();
		Field[] allFields = this.getClass().getDeclaredFields();

		for (Field f: allFields) {
			if (f.getType().equals(Object[].class)) {
				try {
					f.setAccessible(true);
					Object[] array = (Object[]) f.get(this);
					allObjArrays.put(f.getName(), array);
				} catch (IllegalAccessException e) {
					e.printStackTrace();
				}
			}
		}

		return allObjArrays;
	}
}
