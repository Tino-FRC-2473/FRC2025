package frc.robot.logging;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.io.IOException;
import java.util.Optional;
import java.io.File;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.SimConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.hal.AllianceStationID;

public final class MechLogging {
	private Pose3d elevatorStage1;
	private Pose3d elevatorStage2;
	private Pose3d elevatorStage3;
	private Pose3d climberPose;

	private static MechLogging instance = new MechLogging();

	private boolean doesSimRobotHaveCoral = false; // Preloaded 1 -- temporarily false to test
	private boolean isParallelToStation = false;
	private boolean isInStationZone = false;
	private Pose2d drivetrainPose;
	//private Pose2d spherePose;
	// Timer
	private Timer timer = new Timer();
	private ChassisSpeeds drivetrainChassisSpeeds;
	private Rotation2d driveRotation;
	private static final double TEMP_OFFSET = 0.5;

	private MechLogging() {
		elevatorStage1 = new Pose3d();
		elevatorStage2 = new Pose3d();
		elevatorStage3 = new Pose3d();
		climberPose = new Pose3d();
	}

	public static final double STAGE_2_RATIO = 1.0 / 3.0;
	public static final double STAGE_3_RATIO = 2.0 / 3.0;
	double stationLeftX, stationLeftY, stationRightX, stationRightY;

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
	}
	











	/* 	SimulatedArena.getInstance()
				.addGamePieceProjectile(new ReefscapeCoralOnFly(
						// Obtain robot position from drive simulation
						drivetrainPose,
						// The scoring mechanism is installed at (0.46, 0) (meters) on the robot
						new Translation2d(0.35, 0),
						// Obtain robot speed from drive simulation
						drivetrainChassisSpeeds,
						// Obtain robot facing from drive simulation
						driveRotation,
						// The height at which the coral is ejected
						Meters.of(1.28),
						// The initial speed of the coral
						MetersPerSecond.of(2),
						// The coral is ejected at a 35-degree slope
						Degrees.of(-35)));
	}
	*/
	/**
	 * Checks if it is inside the station zone
	 * If it is, checks if it can intake coral
	 * Finally calls dropCoral to pick up the coral.
	 */
	public void intakeCoral() {
		//System.out.println("enterss method");
		if (doesSimRobotHaveCoral()) {
			return;
		}
		if (DriverStationSim.getAllianceStationId().equals(AllianceStationID.Unknown))	{
			return;
		}
		
		
	
		double robotHeading = drivetrainPose.getRotation().getRadians();
		//Converting width to meters to calculate translation
		double widthMeters = SimConstants.WIDTH_IN * 0.0254;
		double translationX = widthMeters * Math.cos(robotHeading);
		double translationY = widthMeters * Math.sin(robotHeading);
		double robotBackX = drivetrainPose.getX() - translationX;
		double robotBackY = drivetrainPose.getY() - translationY;
	
		try {
			File fieldLayoutFile = new File("src/main/java/frc/robot/constants/tagAbsPos.json");
			AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(fieldLayoutFile.getAbsolutePath());
			if (fieldLayout != null) {
				
				Optional<Pose3d> stationLeft;
				Optional<Pose3d> stationRight;
				
				boolean blueAlliance = DriverStationSim.getAllianceStationId().equals(AllianceStationID.Blue1) || DriverStationSim.getAllianceStationId().equals(AllianceStationID.Blue2) || DriverStationSim.getAllianceStationId().equals(AllianceStationID.Blue3);
				if (blueAlliance) {
					//System.out.println("blue");
					stationLeft = fieldLayout.getTagPose(SimConstants.BLUE_STATION_LEFT_APRILTAG_ID);
					stationRight = fieldLayout.getTagPose(SimConstants.BLUE_STATION_RIGHT_APRILTAG_ID);
					double stationLeftX = 0, stationLeftY = 0, stationRightX = 0, stationRightY = 0;
					
					if (stationLeft != null && stationRight != null){
						stationLeftX = stationLeft.get().getX();
						stationLeftY = stationLeft.get().getY();
						stationRightX = stationRight.get().getX();
						stationRightY = stationRight.get().getY();
						double stationAngleLeft = stationLeft.get().getRotation().getZ(); //radians
						double stationAngleRight = stationRight.get().getRotation().getZ();
						//slope of the station
						double leftSlope = -1 * Math.tan(stationAngleLeft);
						double rightSlope = -1 * Math.tan(stationAngleRight);
						//finding the change in x and y of the robot from the station
						double leftXChange = robotBackX - (SimConstants.MAX_DISTANCE_FROM_STATION * Math.cos(stationAngleLeft)) - stationLeftX;
						double leftYChange = leftXChange * leftSlope;
						double rightXChange = robotBackX - (SimConstants.MAX_DISTANCE_FROM_STATION * Math.cos(stationAngleLeft)) - stationRightX;
						double rightYChange = rightXChange * rightSlope;
						//finding the position on the station where the distance check should apply
						double leftYMargin = stationLeftY + leftYChange - (SimConstants.MAX_DISTANCE_FROM_STATION * Math.sin(stationAngleLeft));
						double rightYMargin = stationRightY + rightYChange + (SimConstants.MAX_DISTANCE_FROM_STATION * Math.sin(stationAngleRight));
						//left station check
						if (robotBackX < (SimConstants.CORAL_STATION_WIDTH_METERS * 0.5 * Math.cos(stationAngleLeft + (Math.PI/2)) + stationLeftX) && robotBackX > stationLeftX - (SimConstants.CORAL_STATION_WIDTH_METERS * 0.5 * Math.cos(stationAngleLeft + (Math.PI/2)))){
							if (robotBackY > leftYMargin || robotBackY < rightYMargin){
								isInStationZone = true;
							} else {
								isInStationZone = false;
							}
						} else {
							isInStationZone = false;
						}
						
						double robotAngle = drivetrainPose.getRotation().getRadians();
		
						isParallelToStation = Math.abs(stationAngleLeft - robotAngle) <= Math.toRadians(5) ||
											Math.abs(stationAngleRight - robotAngle) <= Math.toRadians(5);
					}

				} else {
					//System.out.println("red");
					stationLeft = fieldLayout.getTagPose(SimConstants.RED_STATION_LEFT_APRILTAG_ID);
					stationRight = fieldLayout.getTagPose(SimConstants.RED_STATION_RIGHT_APRILTAG_ID);
				
					double stationLeftX = 0, stationLeftY = 0, stationRightX = 0, stationRightY = 0;
					if (stationLeft != null && stationRight != null){
						stationLeftX = stationLeft.get().getX();
						stationLeftY = stationLeft.get().getZ();
						stationRightX = stationRight.get().getX();
						stationRightY = stationRight.get().getZ();
						double stationAngleLeft = stationLeft.get().getRotation().getZ(); //radians
						double stationAngleRight = stationRight.get().getRotation().getZ();
						//slope of the station
						double leftSlope = Math.tan(stationAngleLeft + (Math.PI/2));
						double rightSlope = Math.tan(stationAngleRight + (Math.PI/2));
						//finding the change in x and y of the robot from the station
						double leftXChange = robotBackX - (SimConstants.MAX_DISTANCE_FROM_STATION * Math.cos(stationAngleLeft)) - stationLeftX;
						double leftYChange = leftXChange * leftSlope;
						double rightXChange = robotBackX - (SimConstants.MAX_DISTANCE_FROM_STATION * Math.cos(stationAngleRight)) - stationRightX;
						double rightYChange = rightXChange * rightSlope;
						//finding the position on the station where the distance check should apply
						double leftYMargin = stationLeftY + leftYChange + (SimConstants.MAX_DISTANCE_FROM_STATION * Math.sin(stationAngleLeft));
						double rightYMargin = stationRightY + rightYChange - (SimConstants.MAX_DISTANCE_FROM_STATION * Math.sin(stationAngleRight));

						//left station check
						if (robotBackX < (SimConstants.CORAL_STATION_WIDTH_METERS * 0.5 * Math.cos(stationAngleLeft + (Math.PI)) + stationLeftX) && robotBackX > stationLeftX - (SimConstants.CORAL_STATION_WIDTH_METERS * 0.5 * Math.cos(stationAngleLeft + (Math.PI)))){
							if (robotBackY < leftYMargin || robotBackY > rightYMargin){
								System.out.println("enters true statement");
								isInStationZone = true;
							} else {
								isInStationZone = false;
							}
						} else {
							isInStationZone = false;
						}

						double robotAngle = drivetrainPose.getRotation().getRadians();
		
						isParallelToStation = Math.abs(stationAngleLeft - robotAngle) <= Math.toRadians(5) ||
											Math.abs(stationAngleRight - robotAngle) <= Math.toRadians(5);
					}
				}
				if (isInStationZone && isParallelToStation) {
					System.out.println("Reached #1");
					if (!timer.isRunning()) {  // If the timer isn't running, start it
						System.out.println("Reached #2");
						timer.start();
					} 
					System.out.println("reached here");
					if (timer.hasElapsed(2)){
						System.out.println("VICTORY");
						doesSimRobotHaveCoral = true;
						SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
							new Pose2d(drivetrainPose.getX(), drivetrainPose.getY(), drivetrainPose.getRotation()))
						);
					
						System.out.println("Coral spawned directly on top of the robot.");
						Pose2d spawnPose = new Pose2d(drivetrainPose.getX(), drivetrainPose.getY(), drivetrainPose.getRotation());
						ReefscapeCoralOnField coral = new ReefscapeCoralOnField(spawnPose);

						System.out.println("Attempting to spawn coral at: " + spawnPose);
						SimulatedArena.getInstance().addGamePiece(coral);
						System.out.println("Coral spawn added to SimulatedArena.");
						SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
    					new Pose2d(2, 2, Rotation2d.fromDegrees(90))));

						SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2,2)));

						Logger.recordOutput("FieldSimulation/Algae", 
    					SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
						Logger.recordOutput("FieldSimulation/Coral", 
    					SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));

						

					}
					
				} else {
					if (timer.isRunning()) { 
						System.out.println("FAILURE"); // If the robot is out of the zone, stop the timer
						timer.reset();
					}
				}
			}
		} catch (IOException error) {
			System.out.println(error.getMessage());
		}
	}
	public void updateElevatorOuttake(){
		if(DriverStation.getStickButton(0, 1)){
			Pose3d curPose = getElevatorStage3();
			Pose3d updatedElevatorPose = new Pose3d(
                elevatorStage3.getTranslation().plus(new Translation3d(0, 0, TEMP_OFFSET)),
                elevatorStage3.getRotation()
            );
            Logger.recordOutput("ElevatorPose", updatedElevatorPose);
		}
	}
	/**
	 * Checks if the robot is in the station zone.
	 * @return if the robot is in the station zone
	 */	
	public boolean isInStationZone() {
		return isInStationZone;
	}

	/**
	 * Checks if the robot is parallel to the station.
	 * @return if the robot is parallel to the station
	 */
	public boolean isParallelToStation() {
		return isParallelToStation;
	}



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
		//this.spherePose = spherePose;

	}
}
