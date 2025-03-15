package frc.robot.logging;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

import org.dyn4j.geometry.Transform;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.constants.SimConstants;
import edu.wpi.first.wpilibj.Timer;

public final class MechLogging {
	private Pose3d elevatorStage1;
	private Pose3d elevatorStage2;
	private Pose3d elevatorStage3;
	private Pose3d climberPose;

	private static MechLogging instance = new MechLogging();

	private boolean doesSimRobotHaveCoral = true; // Preloaded 1

	private Pose2d drivetrainPose;
	private ChassisSpeeds drivetrainChassisSpeeds;
	private Rotation2d driveRotation;
	private Timer timer = new Timer();
	private AprilTagFieldLayout layout;

	private MechLogging() {
		elevatorStage1 = new Pose3d();
		elevatorStage2 = new Pose3d();
		elevatorStage3 = new Pose3d();
		climberPose = new Pose3d();
		try{
			File fieldLayoutFile = new File("src/main/java/frc/robot/constants/tagAbsPos.json");
			layout = new AprilTagFieldLayout(fieldLayoutFile.getAbsolutePath());
		}catch(IOException e){
			e.printStackTrace();
		}
	}

	public static final double STAGE_2_RATIO = 1.0 / 3.0;
	public static final double STAGE_3_RATIO = 2.0 / 3.0;

	/**
	 * Get the instance of the singleton class.
	 * @return the instance
	 */
	public static MechLogging getInstance() {
		return instance;
	}

	/**
	 * Drop the coral with the specified pose.
	 */
	public void dropCoral() {
		if (!doesSimRobotHaveCoral) {
			return;
		}
		doesSimRobotHaveCoral = false;

		//TODO: Add maple-sim drop coral functionality here.
	}

	/**
	 * Checks if it is inside the station zone
	 * If it is, checks if it can intake coral
	 * Finally calls dropCoral to pick up the coral.
	 */
	public void intakeCoralIfPossible() {
		if(layout == null)return;
		if(layout != null){
			//System.out.println("not null");
			var inZone = isBackOfRobotInStationZone(layout);
			if(inZone && timer.hasElapsed(2)) {
				doesSimRobotHaveCoral = true;
				timer.stop();
			} else if (inZone && !timer.isRunning()) {
				timer.start();
			} else if(!inZone && timer.isRunning()) {
				timer.stop();
			}
				//System.out.println(isBackOfRobotInStationZone(fieldLayout));
		}
			
			
		}
		
	

	public boolean isBackOfRobotInStationZone(AprilTagFieldLayout layout) {
		var backOfRobotTransform = drivetrainPose.plus(new Transform2d(new Translation2d(SimConstants.WIDTH_IN / 2, SimConstants.LENGTH_IN / 2), driveRotation));
		//if(blue alliance)
		//System.out.println(getAlliance());
		//System.out.println(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue);
		if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
			
			System.out.println(arePosesColinear(12, getAlliance()));
			System.out.println(arePosesColinear(13, getAlliance()));
		} else {
			return false;
		}
		return false;
	}
	private boolean arePosesColinear(int ID, Alliance alliance){
		var pointRight = new Transform2d(
				new Translation2d(
					SimConstants.CORAL_STATION_WIDTH_METERS/2.0,0), 
					layout.getTags().get(ID).pose.getRotation().toRotation2d()
				);
			var pointLeft =  new Transform2d(
				new Translation2d(-SimConstants.CORAL_STATION_WIDTH_METERS/2.0,0),
				layout.getTags().get(ID).pose.getRotation().toRotation2d()
			);

			//Tag at (0,0)
			System.out.println("point right is " + pointRight);
			System.out.println("point right Y is " + pointRight.getY());
			System.out.println("point right X is " + pointRight.getX());
			System.out.println("right - drive" + (pointRight.getX()-drivetrainPose.getX()));
			System.out.println("drive train pose y is " + drivetrainPose.getY());
			double slopeRight = (pointRight.getY() - drivetrainPose.getY())/(pointRight.getX()-drivetrainPose.getX());
			double slopeLeft = (pointLeft.getY() - drivetrainPose.getY())/(pointLeft.getX()-drivetrainPose.getX());
			double slopeStation = (pointRight.getY()-pointLeft.getY())/(pointRight.getX()-pointLeft.getX());
			//System.out.println("slope left is " +slopeLeft);
			//System.out.println("slope right is " + slopeRight);
			//double error = isApproxEqual(slopeStation, slopeRight, 0.2);
			if(isApproxEqual(slopeStation, slopeLeft, 0.5) && isApproxEqual(slopeStation, slopeRight, 0.5) && isApproxEqual(slopeRight, slopeLeft, 0.5)){
				return true;
			}
			return false;
	}
	// private double calculateApproxError(double slopeRight, double slopeLeft, double slopeStation){
	// 	double checkError = Math.abs(slopeStation/slopeLeft);
	// 	return checkError;
	// }

	private boolean isApproxEqual(double v1, double v2, double epsilon) {
		return Math.abs(v1 - v2) < epsilon;
	}

	private Alliance getAlliance() {
		var alliance = DriverStationSim.getAllianceStationId();

		return isAllianceEqualToAny(alliance, AllianceStationID.Blue1, AllianceStationID.Blue2, AllianceStationID.Blue3)
			? Alliance.Blue
			: isAllianceEqualToAny(alliance, AllianceStationID.Red1, AllianceStationID.Red2, AllianceStationID.Red3)
				? Alliance.Red
				: null;
	}

	private boolean isAllianceEqualToAny(AllianceStationID allianceStationID, AllianceStationID... allianceStationIDs) {
		Set<AllianceStationID> allSet = new HashSet<>(Arrays.asList(allianceStationIDs));
		return allSet.contains(allianceStationID);
	}
	
	// need to check if all along the length of the station if two cm away with some degree of error on angle, then  



	/**
	 * Generates the pose for the elevator based on encoder position.
	 * @param encoderSimPosition the simulated location of the elevator motor encoder.
	 */
	public void updateElevatorPose3d(Angle encoderSimPosition) {
		double totalHeight =
			encoderSimPosition
				.div(SimConstants.ELEVATOR_GEAR_RATIO)
				.in(Radians)
			* SimConstants.ELEVATOR_WINCH_DIAMETER_METERS / 2;

		// Stage 1 (bottom stage) moves 1/6 of the total movement
		elevatorStage1 = new Pose3d(
			Pose3d.kZero
				.getTranslation().plus(new Translation3d(0, 0, 0)),
			Rotation3d.kZero
		);

		// Stage 2 (middle stage) moves 1/3 of the total movement
		elevatorStage2 = new Pose3d(
			Pose3d.kZero
				.getTranslation().plus(new Translation3d(0, 0, totalHeight * STAGE_2_RATIO)),
			Rotation3d.kZero
		);

		// Stage 3 (top stage) moves 1/2 of the total movement
		elevatorStage3 = new Pose3d(
			Pose3d.kZero
				.getTranslation()
				.plus(new Translation3d(0, 0, totalHeight * STAGE_3_RATIO)),
			Rotation3d.kZero
		);
	}

	/**
	 * Updates the pose for the climber based on encoder position.
	 * @param encoderSimPosition the simulated location of the climber motor encoder.
	 */
	public void updatesClimberPose3d(Angle encoderSimPosition) {
		climberPose = new Pose3d(
			SimConstants.CLIMBER_ZERO_POS,
			new Rotation3d(
				encoderSimPosition.div(SimConstants.CLIMBER_GEAR_RATIO).in(Radians),
				0,
				0
			)
		);
	}

	/**
	 * Get the primary elevator pose.
	 * @return the pose of the inner part of the elevator
	 */
	public Pose3d getElevatorStage1() {
		return elevatorStage1;
	}

	/**
	 * Get the secondary elevator pose.
	 * @return the pose of the inner-most part of the elevator
	 */
	public Pose3d getElevatorStage2() {
		return elevatorStage2;
	}

	/**
	 * Get the elevator stage 3 position.
	 * @return the elevator stage 3 position
	 */
	public Pose3d getElevatorStage3() {
		return elevatorStage3;
	}

	/**
	 * Get the pose of the climber ligament.
	 * @return pose of the rotating climber ligament.
	 */
	public Pose3d getClimberPose() {
		return climberPose;
	}

	/**
	 * Getter for the array of poses we want to simulate.
	 * @return the array of poses to display in advantageScope
	 */
	public Pose3d[] getRobotPoses() {
		return new Pose3d[]{
			getElevatorStage1(),
			getElevatorStage2(),
			getElevatorStage3(),
			getClimberPose()
		};
	}

	/**
	 * Gets if the sim robot has a coral loaded in the robot.
	 * @return if the sim robot has a coral loaded in the robot
	 */
	public boolean doesSimRobotHaveCoral() {
		return doesSimRobotHaveCoral;
	}

	/**
	 * Updates the drivetrain values.
	 * @param pose the pose of the drivetrain
	 * @param speeds the chassis speeds of the drivetrain
	 * @param rotation the rotation of the drivetrain
	 */
	public void updateDrivetrainValues(Pose2d pose, ChassisSpeeds speeds, Rotation2d rotation) {
		drivetrainPose = pose;
		drivetrainChassisSpeeds = speeds;
		driveRotation = rotation;
	}
}
