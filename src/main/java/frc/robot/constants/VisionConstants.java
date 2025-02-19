package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class VisionConstants {
	public static final int UNABLE_TO_SEE_NOTE_CONSTANT = 5000;
	public static final int UNABLE_TO_SEE_TAG_CONSTANT = 4000;

	public static final double MAX_SPEED_METERS_PER_SECOND = 0.15;
	public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI * 0.1;

	//TODO: equate the margin to the probabilistic equation from HW.
	public static final double X_MARGIN_TO_REEF = 0.02;
	public static final double Y_MARGIN_TO_REEF = 0.02;
	public static final double ROT_MARGIN_TO_REEF
		= Math.toRadians(2); //TODO: change rot margin.
	public static final double TAG_TARGET_DISTANCE = 0.2;

	public static final double N_180 = 180;

	public static final int AT_ARR_INC = 10;
	public static final int AT_ARR_CAMERA_OFFSET = 1;
	public static final int AT_ARR_TRANSLATION_OFFSET = 4;
	public static final int AT_ARR_ROTATION_OFFSET = 7;

	public static final String REEF_CAM_NAME = "Reef Cam";
	public static final String SOURCE_CAM_NAME = "Source Cam";

	public static final String APRIL_TAG_FIELD_LAYOUT_JSON =
		"src/main/java/frc/robot/constants/apriltag/welded/apriltag.json";

	public static final int LOCALIZATION_TAG_NUM = 1;
	public static final double LOCALIZATION_TRANSLATIONAL_THRESHOLD = 0.4;
	public static final double LOCALIZATION_ANGLE_TOLERANCE = Math.toRadians(5);
}
