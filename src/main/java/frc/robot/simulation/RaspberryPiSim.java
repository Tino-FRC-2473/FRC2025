package frc.robot.simulation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.AprilTag;
import frc.robot.RaspberryPi;
import frc.robot.constants.SimConstants;
import frc.robot.constants.VisionConstants;

import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class RaspberryPiSim extends RaspberryPi {
	private static VisionSystemSim visionSim;

	private final PhotonCameraSim reefCameraSim;
	private final PhotonCameraSim stationCameraSim;
	private final PhotonCamera reefCamera;
	private final PhotonCamera stationCamera;

	/**
	* Creates a new RaspberryPiSim.
	*/
	public RaspberryPiSim() {
		reefCamera = new PhotonCamera(SimConstants.REEF_CAMERA_NAME);
		stationCamera = new PhotonCamera(SimConstants.STATION_CAMERA_NAME);

		// Initialize vision sim
		if (visionSim == null) {
			visionSim = new VisionSystemSim("main");
			try {
				visionSim.addAprilTags(
					new AprilTagFieldLayout(VisionConstants.APRIL_TAG_FIELD_LAYOUT_JSON)
				);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}

		// Add sim camera
		var cameraProperties = new SimCameraProperties();
		reefCameraSim = new PhotonCameraSim(reefCamera, cameraProperties);
		stationCameraSim = new PhotonCameraSim(stationCamera, cameraProperties);

		visionSim.addCamera(reefCameraSim, SimConstants.ROBOT_TO_REEF_CAMERA);
		visionSim.addCamera(stationCameraSim, SimConstants.ROBOT_TO_STATION_CAMERA);
	}

	/**
	 * Prints all raw apriltag data to console.
	 */
	public void printRawData() {
		for (AprilTag tag: getAprilTags()) {
			System.out.println("AprilTag " + tag.getTagID() + " -> " + tag.getPose().toString());
		}
	}

	/**
	 * Returns a list of all april tags from reef and station camera.
	 * @return all april tags
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
					target.fiducialId,
					reefCamera.getName(),
					new Translation3d(), //camera vector, unused
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
				// if (at.getPose().getTranslation().getNorm() < SimConstants.CAM_DISTANCE_READ) {
				atList.add(at);
				// }
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
					target.fiducialId,
					stationCamera.getName(),
					new Translation3d(), //camera vector, unused
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
				// if (at.getPose().getTranslation().getNorm() < SimConstants.CAM_DISTANCE_READ) {
				atList.add(at);
				// }
			}
		}
		return atList;
	}

	/**
	 * Updates raspberry pi's simulated pose based on MapleSim pose.
	 * @param robotPoseMeters current pose
	 */
	public void update(Pose2d robotPoseMeters) {
		visionSim.update(robotPoseMeters);
	}
}
