package frc.robot.logging;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.io.IOException;
import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
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

	private MechLogging() {
		elevatorStage1 = new Pose3d();
		elevatorStage2 = new Pose3d();
		elevatorStage3 = new Pose3d();
		climberPose = new Pose3d();
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

	// /**
	//  * Drop the coral with the specified pose.
	//  */
	// public void dropCoral() {
	// 	if (!doesSimRobotHaveCoral) {
	// 		return;
	// 	}
	// 	doesSimRobotHaveCoral = false;
	// }
	











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
	// public void intakeCoral() {
	// 	// Timer
	// 	Timer timer = new Timer();
	// 	timer.start();
	// 	boolean timerRunning = true;
	
	// 	double robotHeading = drivetrainPose.getRotation().getRadians();
	// 	double translationX = SimConstants.WIDTH_IN * Math.cos(robotHeading);
	// 	double translationY = SimConstants.WIDTH_IN * Math.sin(robotHeading);
	// 	double robotBackX = drivetrainPose.getX() - translationX;
	// 	double robotBackY = drivetrainPose.getY() - translationY;
	
	// 	try {
	// 		AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout("../constants/tagAbsPos.json");
	// 		if (fieldLayout != null) {
	// 			if (DriverStation.getAlliance().equals(Alliance.Blue)) {
	// 				Optional<Pose3d> stationLeft = fieldLayout.getTagPose(SimConstants.BLUE_STATION_LEFT_APRILTAG_ID);
	// 				Optional<Pose3d> stationRight = fieldLayout.getTagPose(SimConstants.BLUE_STATION_RIGHT_APRILTAG_ID);
	// 				double stationLeftX = 0, stationLeftY = 0, stationRightX = 0, stationRightY = 0;
	
	// 				if (stationLeft.isPresent()) {
	// 					stationLeftX = stationLeft.get().getX();
	// 					stationLeftY = stationLeft.get().getY();
	// 				}
	// 				if (stationRight.isPresent()) {
	// 					stationRightX = stationRight.get().getX();
	// 					stationRightY = stationRight.get().getY();
	// 				}
	
	// 				// Check if the robot is within 2 centimeters of the station
	// 				double distanceLeft = Math.sqrt(Math.pow(robotBackX - stationLeftX, 2) + Math.pow(robotBackY - stationLeftY, 2));
	// 				double distanceRight = Math.sqrt(Math.pow(robotBackX - stationRightX, 2) + Math.pow(robotBackY - stationRightY, 2));
	// 				boolean isInZone = (distanceLeft <= 0.02 || distanceRight <= 0.02);
	
	// 				double stationAngleLeft = Math.atan2(stationLeftY - robotBackY, stationLeftX - robotBackX);
	// 				double stationAngleRight = Math.atan2(stationRightY - robotBackY, stationRightX - robotBackX);
	// 				double robotAngle = drivetrainPose.getRotation().getRadians();
	
	// 				boolean isParallel = Math.abs(stationAngleLeft - robotAngle) <= Math.toRadians(5) ||
	// 									 Math.abs(stationAngleRight - robotAngle) <= Math.toRadians(5);
	// 				Logger.recordOutput("isParallel", isParallel);
	// 				Logger.recordOutput("inZone", isInZone);


	
	// 				if (isInZone && isParallel) {
	// 					if (!timerRunning) {  // If the timer isn't running, start it
	// 						timer.start();
	// 						timerRunning = true;
	// 					}
	// 				} else {
	// 					if (timerRunning) {  // If the robot is out of the zone, stop the timer
	// 						timer.reset();
	// 						timerRunning = false;
	// 					}
	// 				}
	// 			}
	// 		}
	// 	} catch (IOException error) {
	// 		System.out.println(error.getMessage());
	// 	}
	// }
	
	// // need to check if all along the length of the station if two cm away with some degree of error on angle, then  



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

	// /**
	//  * Updates the pose for the climber based on encoder position.
	//  * @param encoderSimPosition the simulated location of the climber motor encoder.
	//  */
	// public void updatesClimberPose3d(Angle encoderSimPosition) {
	// 	climberPose = new Pose3d(
	// 		SimConstants.CLIMBER_ZERO_POS,
	// 		new Rotation3d(
	// 			encoderSimPosition.div(SimConstants.CLIMBER_GEAR_RATIO).in(Radians),
	// 			0,
	// 			0
	// 		)
	// 	);
	// }

	// /**
	//  * Get the primary elevator pose.
	//  * @return the pose of the inner part of the elevator
	//  */
	// public Pose3d getElevatorStage1() {
	// 	return elevatorStage1;
	// }

	// /**
	//  * Get the secondary elevator pose.
	//  * @return the pose of the inner-most part of the elevator
	//  */
	// public Pose3d getElevatorStage2() {
	// 	return elevatorStage2;
	// }

	// /**
	//  * Get the elevator stage 3 position.
	//  * @return the elevator stage 3 position
	//  */
	// public Pose3d getElevatorStage3() {
	// 	return elevatorStage3;
	// }

	// /**
	//  * Get the pose of the climber ligament.
	//  * @return pose of the rotating climber ligament.
	//  */
	// public Pose3d getClimberPose() {
	// 	return climberPose;
	// }

	// /**
	//  * Getter for the array of poses we want to simulate.
	//  * @return the array of poses to display in advantageScope
	//  */
	// public Pose3d[] getRobotPoses() {
	// 	return new Pose3d[]{
	// 		getElevatorStage1(),
	// 		getElevatorStage2(),
	// 		getElevatorStage3(),
	// 		getClimberPose()
	// 	};
	// }

	// /**
	//  * Gets if the sim robot has a coral loaded in the robot.
	//  * @return if the sim robot has a coral loaded in the robot
	//  */
	// public boolean doesSimRobotHaveCoral() {
	// 	return doesSimRobotHaveCoral;
	// }

	// /**
	//  * Updates the drivetrain values.
	//  * @param pose the pose of the drivetrain
	//  * @param speeds the chassis speeds of the drivetrain
	//  * @param rotation the rotation of the drivetrain
	//  */
	// public void updateDrivetrainValues(Pose2d pose, ChassisSpeeds speeds, Rotation2d rotation) {
	// 	drivetrainPose = pose;
	// 	drivetrainChassisSpeeds = speeds;
	// 	driveRotation = rotation;
	// }
}
