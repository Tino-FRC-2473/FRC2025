package frc.robot;
import java.util.ArrayList;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

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
	 * Print all raw data from the tag subscriber.
	 */
	public void printRawData() {
		double[] rawData = tagSubscriber.get();
		System.out.println(rawData);
	}

	/**
	* Gets the data from the Raspberry Pi.
	*
	* @return  ArrayList<AprilTag>
	*          The data from the Raspberry Pi
	*/
	public ArrayList<AprilTag> getAprilTags() {
		ArrayList<AprilTag> atList = new ArrayList<>();
		double[] rawData = tagSubscriber.get();
		System.out.println(rawData.length);
		if (rawData.length == 0) {
			return atList;
		}

		for (int i = 0; i < rawData.length / 10; i += 10) {
			atList.add(
				new AprilTag(i, "Reef Camera",
				getArraySegment(rawData, i + 1, i + 3),
				getArraySegment(rawData, i + 4, i + 6),
				getArraySegment(rawData, i + 7, i + 9))
			);
		}

		return atList;
	}

	/**
	* Gets a sub-ArrayList from the array.
	*
	* @param   src
	*          The array to get the segment from
	* @param   start
	*          The start index of the segment
	* @param   end
	*          The end index of the segment
	* @return  ArrayList<Double>
	*          The segment of the array as an {@code ArrayList<Double>}
	*/
	public static ArrayList<Double> getArraySegment(double[] src, int start, int end) {
		ArrayList<Double> segment = new ArrayList<>();

		for (int i = start; i <= end; i++) {
			segment.add(src[i]);
		}
		return segment;
	}
}
