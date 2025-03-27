package frc.robot.raspberrypi;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.AprilTag;

/**
 * Interface to handle RaspberryPi classes.
 */
public interface RaspberryPi {
	/**
	 * Prints the raw data for the AprilTags on the Pi.
	 */
	void printRawData();

	/**
	* Returns a list of all AprilTags from all cameras.
	* @return A list of visible AprilTags
	*/
	ArrayList<AprilTag> getAprilTags();

	/**
	 * Get all AprilTags reported from the reef camera.
	 * @return A list of visible AprilTags
	 */
	ArrayList<AprilTag> getReefAprilTags();

	/**
	 * Get all AprilTags reported from the station camera.
	 * @return A list of visible AprilTags
	 */
	ArrayList<AprilTag> getStationAprilTags();

	/**
	 * Gets an AprilTag from the list given a certain tag.
	 * @param id id of the AprilTag
	 * @return the AprilTag matching the id
	 */
	AprilTag getAprilTagWithID(int id);

	/**
	 * Checks if any AprilTags are in view.
	 * @return A boolean representing if any tags are in view
	 */
	boolean canSeeTags();

	/**
	 * Returns the closest AprilTag from any camera.
	 * @return The closest AprilTag object. If none are in view, returns null.
	 */
	AprilTag getClosestTag();

	/**
	 * Updates the raspberry pi's values given the current robot pose.
	 * Not used for teleop functionality.
	 * @param pose current pose
	 */
	void update(Pose2d pose);
}
