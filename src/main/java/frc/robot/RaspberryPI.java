package frc.robot;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.VisionConstants;

public class RaspberryPI {
	private double fps = 0;
	private NetworkTable table;

	private DoubleSubscriber fpsCounter;
	private DoubleArraySubscriber tagSubscriber;
	private double previousValueReceived = 0;
	private double previousTimeReceived = 0;
	private Timer timer = new Timer();
	public static final int VALUES_PER_TAG = 6;

	/**Updates the FPS each iteration of the robot.*/
	public RaspberryPI() {
		timer.start();
		table = NetworkTableInstance.getDefault().getTable("datatable");
		fpsCounter = table.getDoubleTopic("x").subscribe(-1);
		tagSubscriber = table.getDoubleArrayTopic("april_tag_data").subscribe(null);
	}

	/**Updates the values in SmartDashboard. */
	public void update() {
		updateFPS();
	}

	/**
	 * Updates the FPS each iteration of the robot.
	 */
	public void updateFPS() {
		double currentReceivedValue = fpsCounter.get();
		if (currentReceivedValue != previousValueReceived) {
			fps = 1.0 / (timer.get() - previousTimeReceived);
			previousTimeReceived = timer.get();
		}
		previousValueReceived = currentReceivedValue;
		SmartDashboard.putNumber("FPS", fps);
	}

	/**
	 * @param id id of the april tag we are fetching data on
	 * @return X value from the tag to camera in meters
	 * This value is used in tag-relative swerve movements
	 */
	public double getAprilTagX(int id) {
		if (!tagSubscriber.get().equals(null)) {
			return tagSubscriber.get()[(VALUES_PER_TAG * (id - 1))];
		} else {
			return VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT;
		}
	}

	/**
	 * @param id id of the april tag we are fetching data on
	 * @return Y value from the tag to camera in meters
	 * This value is used in tag-relative swerve movements
	 */
	public double getAprilTagY(int id) {
		if (!tagSubscriber.get().equals(null)) {
			return tagSubscriber.get()[(VALUES_PER_TAG * (id - 1)) + 1];
		} else {
			return VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT;
		}
	}

	/**
	 * @param id id of the april tag we are fetching data on
	 * @return Z value from the tag to camera in meters
	 * This value is used in tag-relative swerve movements
	 */
	public double getAprilTagZ(int id) {
		if (!tagSubscriber.get().equals(null)) {
			return tagSubscriber.get()[(VALUES_PER_TAG * (id - 1)) + 2];
		} else {
			return VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT;
		}
	}

	/**
	 * @param id id of the april tag we are fetching data on
	 * @return X value from the camera to tag in meters
	 * This value is proportional to yaw and is used in robot-relative swerve movements
	 */
	public double getAprilTagXInv(int id) {
		if (!tagSubscriber.get().equals(null)) {
			return tagSubscriber.get()[(VALUES_PER_TAG * (id - 1)) + 2 + 1];
		} else {
			return VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT;
		}
	}

	/**
	 * @param id id of the april tag we are fetching data on
	 * @return Y value from the camera to tag in meters
	 * This value is proportional to pitch and is used in robot-relative swerve movements
	 */
	public double getAprilTagYInv(int id) {
		if (!tagSubscriber.get().equals(null)) {
			return tagSubscriber.get()[(VALUES_PER_TAG * (id - 1)) + 2 + 2];
		} else {
			return VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT;
		}
	}

	/**
	 * @param id id of the april tag we are fetching data on
	 * @return Z value from the camera to tag in meters
	 * This value is used in robot-relative swerve movements
	 */
	public double getAprilTagZInv(int id) {
		if (!tagSubscriber.get().equals(null)) {
			return tagSubscriber.get()[(VALUES_PER_TAG * (id - 1)) + 2 + 2 + 1];
		} else {
			return VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT;
		}
	}

}
