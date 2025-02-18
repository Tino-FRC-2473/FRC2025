package frc.robot.constants;

public class DriveConstants {
	public static final double DRIVE_DEADBAND = 0.01;
	public static final double ROTATION_DEADBAND = 0.01;
	public static final double JOYSTICK_DEADBAND = 0.06;

	public static final double MAX_ANGULAR_VELO_RPS = 0.5;

	public static final double STEER_P = 100;
	public static final double STEER_D = 0.5;
	public static final double STEER_S = 0.1;
	public static final double STEER_V = 2.33;

	public static final double DRIVE_P = 0.1;
	public static final double DRIVE_V = 0.124;

	public static final double DRIVE_CURRENT_LIMIT = 40;
	public static final double STEER_CURRENT_LIMIT = 40;

	public static final double SPEED_DAMP_FACTOR = 2;

	public static final double PASSIVE_ROBOT_FWD_M_S = 0.1;
	public static final double SLEW_RATE = 1.5;
}
