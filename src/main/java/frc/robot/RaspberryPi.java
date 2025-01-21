package frc.robot;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RaspberryPi {
	private NetworkTable table;
	private DoubleArraySubscriber tagSubscriber;

    public RaspberryPi() {
		table = NetworkTableInstance.getDefault().getTable("datatable");
		tagSubscriber = table.getDoubleArrayTopic("april_tag_data").subscribe(null);
    }

    public ArrayList<AprilTag> getAprilTags() {
        ArrayList<AprilTag> ATlist = new ArrayList<>();
        double[] rawData = tagSubscriber.get();

        if (rawData.length == 0) return ATlist;
        
        for(int i = 0; i < rawData.length; i++) {
            //ATlist.add(); Need to rewrite how we send values over networktables
        }

        return ATlist;
    }
}
