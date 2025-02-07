package frc.robot;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
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

	/**
	* Default constructor for the RaspberryPi class.
	*/
	public RaspberryPi() {
		reefTable = NetworkTableInstance.getDefault().getTable("reef_table");
		reefCamSubscriber = reefTable.getDoubleArrayTopic("april_tag_data").subscribe(new double[] {});

		sourceTable = NetworkTableInstance.getDefault().getTable("source_table");
		sourceCamSubscriber = sourceTable.getDoubleArrayTopic("april_tag_data").subscribe(new double[] {});
	}

	/**
	 * Prints the raw data for the april tags on the rpi.
	 */
	public void printRawData() {
		double[] reefRawData = reefCamSubscriber.get();
		double[] sourceRawData = sourceCamSubscriber.get();
		System.out.println("Reef Raw Data: " + Arrays.toString(reefRawData) + "\nSource Raw Data: " + Arrays.toString(sourceRawData));
	}

	/**
	* Returns a list of all AprilTags from all cameras
	*
	* @return  ArrayList<AprilTag>
	*          A list of visible AprilTags
	*/
	public ArrayList<AprilTag> getAprilTags() {
		ArrayList<AprilTag> atList = new ArrayList<>();
		atList.addAll(getAprilTagsSingleCamera(reefCamSubscriber, VisionConstants.REEF_CAM_NAME));
		atList.addAll(getAprilTagsSingleCamera(sourceCamSubscriber, VisionConstants.SOURCE_CAM_NAME));
		return atList;
	}

	/**
	* Returns a list of all AprilTags from one camera
	*
	* @return  ArrayList<AprilTag>
	*          A list of visible AprilTags
	*/
	public ArrayList<AprilTag> getAprilTagsSingleCamera(DoubleArraySubscriber camSubscriber, String camName) {
		ArrayList<AprilTag> atList = new ArrayList<>();
		double[] rawData = camSubscriber.get();

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
}
