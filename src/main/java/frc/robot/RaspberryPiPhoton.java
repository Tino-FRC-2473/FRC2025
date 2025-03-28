package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.VisionConstants;

import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real robot using PhotonVision. */
public class RaspberryPiPhoton extends RaspberryPi {
	private final PhotonCamera reefCamera;
	private final PhotonCamera stationCamera;

	/**
	 * Creates a new RaspberryPiPhoton connecting to real cameras.
	 */
	public RaspberryPiPhoton() {
		// Initialize cameras with their network names
		reefCamera = new PhotonCamera(VisionConstants.REEF_CAM_NAME);
		stationCamera = new PhotonCamera(VisionConstants.SOURCE_CAM_NAME);
	}

	/**
	 * Prints all raw apriltag data to console.
	 */
	@Override
	public void printRawData() {
		for (AprilTag tag : getAprilTags()) {
			System.out.println("AprilTag " + tag.getTagID() + " -> " + tag.getPose().toString());
		}
	}

	/**
	 * Returns a list of all april tags from reef and station camera.
	 * @return all april tags
	 */
	@Override
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
	@Override
	public ArrayList<AprilTag> getReefAprilTags() {
		ArrayList<AprilTag> atList = new ArrayList<>();

		var results = reefCamera.getLatestResult();
		if (results.hasTargets()) {
			System.out.println(results.getTargets().size());
			for (PhotonTrackedTarget target : results.getTargets()) {
				System.out.println(target.getArea());
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
				atList.add(at);
			}
		}
		return atList;
	}

	/**
	 * Returns all april tags visible from Station CV Camera.
	 * @return list of all april tags
	 */
	@Override
	public ArrayList<AprilTag> getStationAprilTags() {
		ArrayList<AprilTag> atList = new ArrayList<>();

		var results = stationCamera.getLatestResult();
		if (results.hasTargets()) {
			for (PhotonTrackedTarget target : results.getTargets()) {
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
				// if (at.getPose().getTranslation().getNorm()
					// < VisionConstants.MAX_TAG_TARGET_DISTANCE_X) {
				atList.add(at);
				// }
			}
		}
		return atList;
	}

	/**
	 * Updates raspberry pi's state based on current robot pose.
	 * Not used for teleop functionality.
	 * @param robotPoseMeters current pose
	 */
	@Override
	public void update(Pose2d robotPoseMeters) {
		// pass
	}
}
