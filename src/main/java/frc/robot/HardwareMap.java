package frc.robot;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int CAN_ID_ELEVATOR = 5;

	// sensor ports

	public static final int ELEVATOR_LIMIT_SWITCH_PORT = 0;

	/* ===== Hardware Availability ===== */
	/**
	 * Check if drive hardware is available to the RoboRIO.
	 * @return true if drive hardware is present
	 */
	public static boolean isDriveHardwarePresent() {
		return true;
	}

	/**
	 * Check if elevator hardware is available to the RoboRIO.
	 * @return true if elevator hardware is present
	 */
	public static boolean isElevatorHardwarePresent() {
		return true;
	}

	/**
	 * Check if funnel hardware is available to the RoboRIO.
	 * @return true if funnel hardware is present
	 */
	public static boolean isFunnelHardwarePresent() {
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
