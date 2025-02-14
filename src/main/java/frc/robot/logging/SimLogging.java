package frc.robot.logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.SimConstants;
import frc.robot.simulation.MapleSimSwerveDrivetrain;

public final class SimLogging {

	private static final SimLogging INSTANCE = new SimLogging();

	private Pose2d simRobotPose = new Pose2d();
	private ChassisSpeeds simRobotChassisSpeeds = new ChassisSpeeds();

	private SimLogging() {}

	/**
	 * Get the instance of the singleton class.
	 * @return the instance
	 */
	public static SimLogging getInstance() {
		return INSTANCE;
	}

	/**
	 * Checks if the robot has hit the station corresponding to the alliance.
	 * @return if the robot can be given a coral
	 */
	public static boolean shouldGiveCoral() {
		isPoseOnLine(new Pose2d(), new Pose2d(), new Pose2d());
		isPoseOnLineSlope(new Pose2d(), new Pose2d(), new Pose2d());
		return true;
	}

	private static boolean isPoseOnLine(Pose2d pose, Pose2d pointA, Pose2d pointB) {
		var start = System.currentTimeMillis();

		double aX = pointA.getX();
		double aY = pointA.getY();
		double bX = pointB.getX();
		double bY = pointB.getY();
		double pX = pose.getX();
		double pY = pose.getY();

		// Check if all the poses are on the same line.
		double crossProd = (pY - aY) * (bX - aX) - (pX - aX) * (bY - aY);
		if (Math.abs(crossProd) > SimConstants.N_1EN9 /* 1e-9 */) {
			return false;
		}

		// Check if the pose is in between pointA and pointB
		double dotProd = (pX - aX) * (bX - aX) + (pY - aY) * (bY - aY);
		double squaredLength = Math.pow(bX - aX, 2) + Math.pow(bY - aY, 2);

		var end = System.currentTimeMillis();

		System.out.println(
			"Time it takes to check if a pose is on the line using dots and crosses: " +
			(end - start)
		);

		// Combine both checks
		return dotProd >= 0 && dotProd <= squaredLength;
	}

	private static boolean isPoseOnLineSlope(Pose2d pose, Pose2d pointA, Pose2d pointB) {
		var start = System.currentTimeMillis();

		var slopeA2B = getSlopeBetweenPoses(pointA, pointB);
		var slopeP2A = getSlopeBetweenPoses(pose, pointA);
		var slopeP2B = getSlopeBetweenPoses(pose, pointB);

		var end = System.currentTimeMillis();

		System.out.println("Time it takes if its slopes" + (end - start));

		return slopeA2B == slopeP2A && slopeP2A == slopeP2B;
	}

	private static double getSlopeBetweenPoses(Pose2d poseA, Pose2d poseB) {
		return (poseB.getY() - poseA.getY()) / (poseB.getX() - poseA.getX());
	}

	/**
	 * Get the simulated robot pose.
	 * @return the simulated robot pose
	 */
	public Pose2d getSimRobotPose() {
		return simRobotPose;
	}

	/**
	 * Apply the logging based on the current state of the simulated drivetrain.
	 * @param simSwerveDrivetrain
	 */
	public void applySimLogging(MapleSimSwerveDrivetrain simSwerveDrivetrain) {
		simRobotPose = simSwerveDrivetrain.getDriveSimulation().getSimulatedDriveTrainPose();
		simRobotChassisSpeeds = simSwerveDrivetrain
				.getDriveSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative();
	}

	/**
	 * Get the simulated robot chassis speeds.
	 * @return the simulated robot chassis speeds
	 */
	public ChassisSpeeds getSimRobotChassisSpeeds() {
		return simRobotChassisSpeeds;
	}
}
