package frc.robot;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
* This class is used to store the pose of the AprilTag relative to the camera.
*/
public class AprilTag {
	private String camera;
	private int tagID;

	/* Describes the orentation of the marker relative to the camera. */
	private Rotation3d rotationalVector;

	/* This is where the tag is positioned relative to the camera. */
	private Translation3d translationalVector;

	/**
	* Constructor for the AprilTag class.
	*
	* @param   id
	*          The ID of the tag
	* @param   camName
	*          The name of the camera
	* @param   rotVector
	*          The orientation of the tag relative to the camera
	* @param   transVector
	*          The position of the tag relative to the camera
	*/
	public AprilTag(
		int id,
		String camName,
		Translation3d transVector,
		Rotation3d rotVector) {

		this.tagID = id;
		this.camera = camName;
		this.rotationalVector = rotVector;
		this.translationalVector = transVector;

	}

	/**
	* Gets the pose of the tag relative to the camera.
	*
	* @return  Pose3d
	*          The pose of the tag relative to the camera
	*/
	public Pose3d getPose() {
		return new Pose3d(translationalVector, rotationalVector);
	}

	/**
	* Get the X position of the tag.
	* @return The {@code double} X position of the tag moving to the right and left
	*/
	public Double getX() {
		return translationalVector.getX();
	}

	/**
	* Get the Y position of the tag.
	* @return The {@code double} Y position of the tag moving up and down, points down
	*/
	public Double getY() {
		return translationalVector.getY();
	}

	/**
	* Get the Z position of the tag.
	* @return The {@code double} Z position of the tag or the forward direction
	*/
	public Double getZ() {
		return translationalVector.getZ();
	}

	/**
	* Get the roll of the tag.
	* @return The {@code double} roll of the tag in radians
	*/
	public Double getRoll() {
		return rotationalVector.getX();
	}

	/**
	* Get the yaw of the tag.
	* @return The {@code double} yaw of the tag in radians
	*/
	public Double getYaw() {
		return rotationalVector.getZ();
	}

	/**
	* Get the pitch of the tag.
	* @return The {@code double} pitch of the tag in radians
	*/
	public Double getPitch() {
		return rotationalVector.getY();
	}

	/**
	* Gets the name of the camera.
	* @return The {@code String} name of the camera
	*/
	public String getCameraName() {
		return camera;
	}

	/**
	* Gets the ID of the tag.
	* @return The {@code int} ID of the tag
	*/
	public int getTagID() {
		return tagID;
	}

	/**
	* Gets string of position values x,y,z and also pose of image.
	* @return The{@code String} position with coordinates and pose
	*/
	@Override
	public String toString() {
		return String.format(
			"ID %d  - x: %.3f, y: %.3f, z: %.3f, %s", tagID, getX(), getY(), getZ(), getPose()
		);
	}
}
