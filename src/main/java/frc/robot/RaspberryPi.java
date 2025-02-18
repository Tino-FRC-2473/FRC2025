package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.VisionConstants;

/**
* This class is used to get the data from the Raspberry Pi.
*
* @author Jaseer Abdulla
*/
public class RaspberryPi {
	private NetworkTable reefTable;
	private NetworkTable sourceTable;
	private DoubleArraySubscriber reefCamSubscriber;
	private DoubleArraySubscriber sourceCamSubscriber;
	private final String reefCamName;
	private final String sourceCamName;

	/**
	* Default constructor for the RaspberryPi class.
	*/
	public RaspberryPi() {
		reefTable = NetworkTableInstance.getDefault().getTable("reef_table");
		DoubleArrayTopic reefCamTopic = reefTable.getDoubleArrayTopic("april_tag_data");
		reefCamSubscriber = reefCamTopic.subscribe(new double[] {});

		sourceTable = NetworkTableInstance.getDefault().getTable("source_table");
		DoubleArrayTopic sourceCamTopic = sourceTable.getDoubleArrayTopic("april_tag_data");
		sourceCamSubscriber = sourceCamTopic.subscribe(new double[] {});

		reefCamName = VisionConstants.REEF_CAM_NAME;
		sourceCamName = VisionConstants.SOURCE_CAM_NAME;
	}

	/**
	 * Prints the raw data for the april tags on the rpi.
	 */
	public void printRawData() {
		double[] reefRawData = reefCamSubscriber.get();
		double[] sourceRawData = sourceCamSubscriber.get();
		System.out.println("Reef Raw Data: " + Arrays.toString(reefRawData));
		System.out.println("Source Raw Data: " + Arrays.toString(sourceRawData));
	}

	/**
	* Returns a list of all AprilTags from all cameras.
	*
	* @return  ArrayList<AprilTag>
	*          A list of visible AprilTags
	*/
	public ArrayList<AprilTag> getAprilTags() {
		ArrayList<AprilTag> atList = new ArrayList<>();
		atList.addAll(getAprilTagsSingleCam(reefCamSubscriber, reefCamName));
		atList.addAll(getAprilTagsSingleCam(sourceCamSubscriber, sourceCamName));
		return atList;
	}

	/**
	* Returns a list of all AprilTags from one camera.
	* @param camSub subscriber for the camera
	* @param camName camera name
	* @return  ArrayList<AprilTag>
	*          A list of visible AprilTags
	*/
	public ArrayList<AprilTag> getAprilTagsSingleCam(DoubleArraySubscriber camSub, String camName) {
		ArrayList<AprilTag> atList = new ArrayList<>();
		double[] rawData = camSub.get();

		if (rawData.length == 0) {
			return atList;
		}

		for (
			int i = 0;
			i < rawData.length;
			i += VisionConstants.AT_ARR_INC
		) {
			atList.add(
				new AprilTag(
					(int) rawData[i],
					camName,
					new Translation3d(
						rawData[i + VisionConstants.AT_ARR_CAMERA_OFFSET],
						rawData[i + VisionConstants.AT_ARR_CAMERA_OFFSET + 1],
						rawData[i + VisionConstants.AT_ARR_CAMERA_OFFSET + 2]
					),
					new Translation3d(
						rawData[i + VisionConstants.AT_ARR_TRANSLATION_OFFSET],
						rawData[i + VisionConstants.AT_ARR_TRANSLATION_OFFSET + 1],
						rawData[i + VisionConstants.AT_ARR_TRANSLATION_OFFSET + 2]
					),
					new Rotation3d(
						rawData[i + VisionConstants.AT_ARR_ROTATION_OFFSET],
						rawData[i + VisionConstants.AT_ARR_ROTATION_OFFSET + 1],
						rawData[i + VisionConstants.AT_ARR_ROTATION_OFFSET + 2]
					)
				)
			);
		}
		return atList;
	}

	/**
	 * Gets an April Tag from the list given a certain tag.
	 * @param id id of the april tag
	 * @return the april tag matching the id
	 */
	public AprilTag getAprilTagWithID(int id) {
		return getAprilTags()
			.stream()
			.filter(tag -> tag.getTagID() == id)
			.findFirst()
			.orElse(null);
	}

	/**
	 * Get all april tags reported from the station camera.
	 * @return list of all apriltags
	 */
	public ArrayList<AprilTag> getStationAprilTags() {
		return getAprilTagsSingleCam(sourceCamSubscriber, VisionConstants.SOURCE_CAM_NAME);
	}

	/**
	 * Get all april tags reported from the reef camera.
	 * @return list of all april tags
	 */
	public ArrayList<AprilTag> getReefAprilTags() {
		return getAprilTagsSingleCam(reefCamSubscriber, VisionConstants.REEF_CAM_NAME);
	}

	/**
	 * Checks if any AprilTags are in view.
	 * @return A boolean representing if any tags are in view
	 */
	public boolean canSeeTags() {
		return getAprilTags().size() != 0;
	}

	/**
	 * Returns the closest AprilTag from any camera.
	 * @return The closest AprilTag object. If none are in view, returns null.
	 */
	public AprilTag getClosestTag() {
		ArrayList<AprilTag> atlist = getAprilTags();
		if (getAprilTags().size() == 0) {
			return null;
		}
		return Collections.max(atlist);
	}

	/**
	 * Updates the raspberry pi's values given the current robot pose.
	 * Not used for teleop functionality.
	 * @param pose
	 */
	public void update(Pose2d pose) {
		// pass
	}
}
