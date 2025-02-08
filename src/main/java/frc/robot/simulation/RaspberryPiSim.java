package frc.robot.simulation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.AprilTag;
import frc.robot.RaspberryPi;
import frc.robot.constants.SimConstants;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
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
					new AprilTagFieldLayout(SimConstants.APRIL_TAG_FIELD_LAYOUT_JSON)
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

	public void printRawData() {
		for (AprilTag tag: getAprilTags()) {
			System.out.println("AprilTag " + tag.getTagID() + " -> " + tag.getPose().toString());
		}
	}

	public ArrayList<AprilTag> getAprilTags() {
		ArrayList<AprilTag> atList = new ArrayList<>();
		atList.addAll(getAprilTagsSingleCamera(reefCamera));
		atList.addAll(getAprilTagsSingleCamera(stationCamera));
		return atList;
	}

	public ArrayList<AprilTag> getReefAprilTags() {
		return getAprilTagsSingleCamera(reefCamera);
	}

	public ArrayList<AprilTag> getStationAprilTags() {
		return getAprilTagsSingleCamera(stationCamera);
	}

	public ArrayList<AprilTag> getAprilTagsSingleCamera(PhotonCamera camera) {
		ArrayList<AprilTag> atList = new ArrayList<>();

		var results = camera.getLatestResult();
		if (results.hasTargets()) {
			for (var target: results.getTargets()) {
				AprilTag at = new AprilTag(
					target.fiducialId,
					camera.getName(),
					new Translation3d(), //camera vector, unused
					new Translation3d(
						target.getBestCameraToTarget().getY(),
						target.getBestCameraToTarget().getZ(),
						target.getBestCameraToTarget().getX()
					),
					new Rotation3d(
						target.getBestCameraToTarget().getRotation()
							.plus(new Rotation3d(Rotation2d.kPi)).getY(),
						target.getBestCameraToTarget().getRotation()
							.plus(new Rotation3d(Rotation2d.kPi)).getZ(),
						target.getBestCameraToTarget().getRotation()
							.plus(new Rotation3d(Rotation2d.kPi)).getX()
					)
				);
				atList.add(at);
			}
		}
		return atList;
	}

	public void update(Pose2d robotPoseMeters) {
		visionSim.update(robotPoseMeters);
	}
}
