package frc.robot.constants;

public class VisionConstants {
	public static final int UNABLE_TO_SEE_NOTE_CONSTANT = 5000;
	public static final int UNABLE_TO_SEE_TAG_CONSTANT = 4000;

	public static final double MAX_SPEED_METERS_PER_SECOND = 0.4;
	public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI / 20;

	public static final double TRANSLATIONAL_ACCEL_CONSTANT = 1.5;
	public static final double ROTATIONAL_ACCEL_CONSTANT = 1.25;
	//TODO: equate the margin to the probabilistic equation from HW.
	public static final double X_MARGIN_TO_REEF = 0.04;
	public static final double Y_MARGIN_TO_REEF = 0.04;
	public static final double ROT_MARGIN_TO_REEF = 0.05;

	public static final double N_180 = 180;

	public static final int AT_ARR_INC = 10;
	public static final int AT_ARR_CAMERA_OFFSET = 1;
	public static final int AT_ARR_TRANSLATION_OFFSET = 4;
	public static final int AT_ARR_ROTATION_OFFSET = 7;
}
