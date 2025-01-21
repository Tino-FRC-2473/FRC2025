package frc.robot;
import java.util.ArrayList;
import edu.wpi.first.math.geometry.*;


public class AprilTag {
    private String camera;
    private int tagID;
    ArrayList<Double> cameraVector; // This is where the camera is positioned relative to the tag
    ArrayList<Double> rotationalVector; // Describes the orentation of the marker relative to the camera
    ArrayList<Double> translationalVector; // This is where the tag is positioned relative to the camera

    public AprilTag(int tagID, String camera, ArrayList<Double> cameraVector, ArrayList<Double> rotationalVector, ArrayList<Double> translationalVector) {
        this.tagID = tagID;
        this.camera = camera;
        this.rotationalVector = rotationalVector;
        this.cameraVector = cameraVector;
        this.translationalVector = translationalVector;
    }

    //Gets the pose of the tag relative to the camera
    public Pose3d getPose() {
        Translation3d trans3d = new Translation3d(translationalVector.get(0), translationalVector.get(1), translationalVector.get(2));
        Rotation3d rot3d = new Rotation3d(rotationalVector.get(0), rotationalVector.get(1), rotationalVector.get(2));
        return new Pose3d(trans3d, rot3d);
    }
    
    public String getCameraName() {
        return camera;
    }

    public int getTagID() {
        return tagID;
    }

}