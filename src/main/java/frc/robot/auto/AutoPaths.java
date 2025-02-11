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
		AutoCommands.B_ALIGN_REEF3_TAG_CMD,
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

	public static final Object[] ELEVATOR_TEST_PATH = new Object[] {
		"No_Path",
		AutoCommands.ELEVATOR_L4_CMD,
		AutoCommands.WAIT,
		AutoCommands.ELEVATOR_L2_CMD,
		AutoCommands.WAIT,
		AutoCommands.ELEVATOR_GROUND_CMD,
		AutoCommands.WAIT,
		AutoCommands.ELEVATOR_L3_CMD,
		AutoCommands.WAIT,
		AutoCommands.ELEVATOR_GROUND_CMD
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
