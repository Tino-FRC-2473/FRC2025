package frc.robot;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int CAN_ID_SPARK_DRIVE_FRONT_RIGHT = 1;
	public static final int CAN_ID_SPARK_DRIVE_BACK_RIGHT = 2;
	public static final int CAN_ID_SPARK_DRIVE_FRONT_LEFT = 3;
	public static final int CAN_ID_SPARK_DRIVE_BACK_LEFT = 4;
	public static final int CAN_ID_ELEVATOR = 5;

	// sensor ports

	public static final int PORT_ELEVATOR_LIMIT_SWITCH = 0;

	/* ===== Hardware Availability ===== */
	/**
	 * Check if drive hardware is available to the RoboRIO.
	 * @return true if drive hardware is present
	 */
	public static boolean isDriveHardwarePresent() {
		return true;
	}

	/**
	 * Check if mech 1 hardware is available to the RoboRIO.
	 * @return true if mech hardware is present
	 */
	public static boolean isMech1HardwarePresent() {
		return false;
	}

	/**
	 * Check if mech 2 hardware is available to the RoboRIO.
	 * @return true if mech 2 hardware is present
	 */
	public static boolean isMech2HardwarePresent() {
		return false;
	}

	/**
	 * Check if drive hardware is available to the RoboRIO.
	 * @return true if drive hardware is present
	 */
	public static boolean isCVHardwarePresent() {
		return false;
	}
}
