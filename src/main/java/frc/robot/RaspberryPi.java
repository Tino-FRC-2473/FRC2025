package frc.robot;

import java.util.ArrayList;

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
	private NetworkTable table;
	private DoubleArraySubscriber tagSubscriber;

	/**
	 * Default constructor for the RaspberryPi class.
	 */
	public RaspberryPi() {
		table = NetworkTableInstance.getDefault().getTable("datatable");
		tagSubscriber = table.getDoubleArrayTopic("april_tag_data").subscribe(new double[] {});
	}

	/**
	 * Prints the raw data for the april tags on the rpi.
	 */
	public void printRawData() {
		double[] rawData = tagSubscriber.get();
		System.out.println(rawData);
	}

	/**
	 * Gets the data from the Raspberry Pi.
	 *
	 * @return ArrayList<AprilTag>
	 *         The data from the Raspberry Pi
	 */
	public ArrayList<AprilTag> getAprilTags() {
		ArrayList<AprilTag> atList = new ArrayList<>();
		double[] rawData = tagSubscriber.get();
		System.out.println(rawData.length);

		if (rawData.length == 0) {
			return atList;
		}

		for (int i = 0; i < rawData.length / VisionConstants.AT_ARR_INC;
			i += VisionConstants.AT_ARR_INC) {
			atList.add(
					new AprilTag(i,
							"Reef Camera",
							getArraySegment(
									rawData,
									i + VisionConstants.AT_ARR_SEG1_START,
									i + VisionConstants.AT_ARR_SEG1_START
										+ VisionConstants.AT_ARR_SEG_LEN),
							getArraySegment(
									rawData,
									i + VisionConstants.AT_ARR_SEG2_START,
									i + VisionConstants.AT_ARR_SEG2_START
									+ VisionConstants.AT_ARR_SEG_LEN),
							getArraySegment(
									rawData,
									i + VisionConstants.AT_ARR_SEG3_START,
									i + VisionConstants.AT_ARR_SEG3_START
									+ VisionConstants.AT_ARR_SEG_LEN)));
		}

		return atList;
	}

	/**
	 * Gets an April Tag from the list given a certain tag.
	 *
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
	 * Gets a sub-ArrayList from the array.
	 *
	 * @param src
	 *              The array to get the segment from
	 * @param start
	 *              The start index of the segment
	 * @param end
	 *              The end index of the segment
	 * @return ArrayList<Double>
	 *         The segment of the array as an {@code ArrayList<Double>}
	 */
	public static ArrayList<Double> getArraySegment(double[] src, int start, int end) {
		ArrayList<Double> segment = new ArrayList<>();

		for (int i = start; i <= end; i++) {
			segment.add(src[i]);
		}

		return segment;
	}
}
