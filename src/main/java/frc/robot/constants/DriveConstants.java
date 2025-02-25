package frc.robot.constants;

public class DriveConstants {
	public static final double DRIVE_DEADBAND = 0.02;
	public static final double ROTATION_DEADBAND = 0.005;
	public static final double JOYSTICK_DEADBAND = 0.1;

	public static final double MAX_ANGULAR_VELO_RPS = 0.5;

	public static final double STEER_P = 100;
	public static final double STEER_D = 0.5;
	public static final double STEER_S = 0.1;
	public static final double STEER_V = 2.33;

	public static final double DRIVE_P = 0.1;
	public static final double DRIVE_V = 0.124;

	public static final double DRIVE_CURRENT_LIMIT = 120;
	public static final double STEER_CURRENT_LIMIT = 40;

	public static final double SPEED_DAMP_FACTOR = 6;

	public static final double PASSIVE_ROBOT_FWD_M_S = 0.1;
	public static final double SLEW_RATE = 2.9;

	public static final double DRIVE_POSE_CHECK_TIMER = 0.5;
	public static final double DRIVE_POSE_CHECK_LP = 1e-3; // LP = LAST POSE
}
