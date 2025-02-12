package frc.robot.logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.simulation.MapleSimSwerveDrivetrain;

public final class SimLogging {

	private static final SimLogging INSTANCE = new SimLogging();

	private Pose2d        simRobotPose          = new Pose2d();
	private ChassisSpeeds simRobotChassisSpeeds = new ChassisSpeeds();


	private SimLogging() {

	}

	/**
	 * Get the instance of the singleton class.
	 * @return the instance
	 */
	public static SimLogging getInstance() {
		return INSTANCE;
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
