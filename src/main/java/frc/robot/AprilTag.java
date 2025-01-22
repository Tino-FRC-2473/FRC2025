package frc.robot;
import java.util.ArrayList;
import edu.wpi.first.math.geometry.*;


/**
 * This class is used to store the pose of the AprilTag relative to the camera
 *
 * @author Jaseer Abdulla
 */
public class AprilTag {
    private String camera;
    private int tagID;
    ArrayList<Double> cameraVector; // This is where the camera is positioned relative to the tag
    ArrayList<Double> rotationalVector; // Describes the orentation of the marker relative to the camera
    ArrayList<Double> translationalVector; // This is where the tag is positioned relative to the camera

    /**
     * Constructor for the AprilTag class
     *
     * @param   tagID
     *          The ID of the tag
     * @param   camera
     *          The name of the camera
     * @param   cameraVector
     *          The position of the camera relative to the tag
     * @param   rotationalVector
     *          The orientation of the tag relative to the camera
     * @param   translationalVector
     *          The position of the tag relative to the camera
     */
    public AprilTag(int tagID, String camera, ArrayList<Double> cameraVector, ArrayList<Double> rotationalVector, ArrayList<Double> translationalVector) {
        this.tagID = tagID;
        this.camera = camera;
        this.rotationalVector = rotationalVector;
        this.cameraVector = cameraVector;
        this.translationalVector = translationalVector;
    }

    /**
     * Gets the pose of the tag relative to the camera
     *
     * @return  Pose3d
     *          The pose of the tag relative to the camera
     */
    public Pose3d getPose() {
        Translation3d trans3d = new Translation3d(translationalVector.get(0), translationalVector.get(1), translationalVector.get(2));
        Rotation3d rot3d = new Rotation3d(rotationalVector.get(0), rotationalVector.get(1), rotationalVector.get(2));
        return new Pose3d(trans3d, rot3d);
    }

    /**
     * Gets the name of the camera
     * @return The {@code String} name of the camera
     */
    public String getCameraName() {
        return camera;
    }

    /**
     * Gets the ID of the tag
     * @return The {@code int} ID of the tag
     */
    public int getTagID() {
        return tagID;
    }

}