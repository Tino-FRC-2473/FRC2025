package frc.robot;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is used to get the data from the Raspberry Pi
 *
 * @author Jaseer Abdulla
 */
public class RaspberryPi {
	private NetworkTable table;
	private DoubleArraySubscriber tagSubscriber;

    /**
     * Default constructor for the RaspberryPi class
     */
    public RaspberryPi() {
		table = NetworkTableInstance.getDefault().getTable("datatable");
		tagSubscriber = table.getDoubleArrayTopic("april_tag_data").subscribe(null);
    }

    public void printRawData() {
        double[] rawData = tagSubscriber.get();
        System.out.println(rawData);
    }

    /**
     * Gets the data from the Raspberry Pi
     *
     * @return  ArrayList<AprilTag>
     *          The data from the Raspberry Pi
     */
    public ArrayList<AprilTag> getAprilTags() {
        ArrayList<AprilTag> ATlist = new ArrayList<>();
        double[] rawData = tagSubscriber.get();

        if (rawData.length == 0) return ATlist;
        
        for(int i = 0; i < rawData.length/10; i++) {
            ATlist.add(new AprilTag(i, "Reef Camera", getArraySegment(rawData, i + 1, i + 3), getArraySegment(rawData, i+4, i+6), getArraySegment(rawData, i + 7, i + 10)));
        }

        return ATlist;
    }

    /**
     * Gets a sub-ArrayList from the array
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

        for(int i = start; i <= end; i++) {
            segment.add(src[i]);
        }
        return segment;
    }
}
