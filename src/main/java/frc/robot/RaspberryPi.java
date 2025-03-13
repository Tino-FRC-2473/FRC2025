package frc.robot;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.VisionConstants;

import org.photonvision.PhotonCamera;

/**
* This class is used to get the data from the Raspberry Pi using PhotonVision.
*/
public class RaspberryPi {
	private final PhotonCamera reefCamera;
	private final PhotonCamera stationCamera;

	/**
	* Default constructor for the RaspberryPi class.
	*/
	public RaspberryPi() {
		reefCamera = new PhotonCamera(VisionConstants.REEF_CAM_NAME);
		stationCamera = new PhotonCamera(VisionConstants.SOURCE_CAM_NAME);
	}

	/**
	 * Prints the raw data for the april tags on the rpi.
	 */
	public void printRawData() {
		for (AprilTag tag : getAprilTags()) {
			System.out.println("AprilTag " + tag.getTagID() + " -> " + tag.getPose().toString());
		}
	}

	/**
	* Returns a list of all AprilTags from all cameras.
	* @return A list of visible AprilTags
	*/
	public ArrayList<AprilTag> getAprilTags() {
		ArrayList<AprilTag> atList = new ArrayList<>();
		atList.addAll(getReefAprilTags());
		atList.addAll(getStationAprilTags());
		return atList;
	}

	/**
	 * Returns a list of all april tags from reef CV camera.
	 * @return all visible reef april tags.
	 */
	public ArrayList<AprilTag> getReefAprilTags() {
		ArrayList<AprilTag> atList = new ArrayList<>();

		var results = reefCamera.getLatestResult();
		if (results.hasTargets()) {
			for (var target: results.getTargets()) {
				AprilTag at = new AprilTag(
					target.getFiducialId(),
					reefCamera.getName(),
					new Translation3d(), // camera vector, unused
					new Translation3d(
						target.getBestCameraToTarget().getY(),
						target.getBestCameraToTarget().getZ(),
						target.getBestCameraToTarget().getX()
					),
					new Rotation3d(
						target.getBestCameraToTarget().getRotation().getY(),
						target.getBestCameraToTarget().getRotation().getZ(),
						target.getBestCameraToTarget().getRotation().getX()
					)
				);
				if (at.getPose().getTranslation().getNorm() < VisionConstants.MAX_TAG_TARGET_DISTANCE_X) {
					atList.add(at);
				}
			}
		}
		return atList;
	}

	/**
	 * Returns all april tags visible from Station CV Camera.
	 * @return list of all april tags
	 */
	public ArrayList<AprilTag> getStationAprilTags() {
		ArrayList<AprilTag> atList = new ArrayList<>();

		var results = stationCamera.getLatestResult();
		if (results.hasTargets()) {
			for (var target: results.getTargets()) {
				AprilTag at = new AprilTag(
					target.getFiducialId(),
					stationCamera.getName(),
					new Translation3d(), // camera vector, unused
					new Translation3d(
						-target.getBestCameraToTarget().getY(),
						-target.getBestCameraToTarget().getZ(),
						-target.getBestCameraToTarget().getX()
					),
					new Rotation3d(
						target.getBestCameraToTarget().getRotation().getY(),
						target.getBestCameraToTarget().getRotation().getZ(),
						target.getBestCameraToTarget().getRotation().getX()
					)
				);
				if (at.getPose().getTranslation().getNorm() < VisionConstants.MAX_TAG_TARGET_DISTANCE_X) {
					atList.add(at);
				}
			}
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
		if (atlist.size() == 0) {
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