package frc.robot.logging;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.concurrent.atomic.AtomicBoolean;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.SimConstants;
import frc.robot.simulation.MapleSimSwerveDrivetrain;

public final class SimLogging {
	private static final SimLogging INSTANCE = new SimLogging();

	private Pose2d simRobotPose = new Pose2d();
	private ChassisSpeeds simRobotChassisSpeeds = new ChassisSpeeds();
	private boolean robotHasCoral = false;

	private SimLogging() { }

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
	public boolean shouldGiveCoral() {
		var shouldGive = new AtomicBoolean(false);

		DriverStation.getAlliance().ifPresent(alliance -> {
			var backPose = new Pose2d(
				alliance == Alliance.Blue
					? SimConstants.STATION_BLUE_LEFT_BACK_X
					: SimConstants.STATION_RED_LEFT_BACK_X,
				alliance == Alliance.Blue
					? SimConstants.STATION_BLUE_LEFT_BACK_X
					: SimConstants.STATION_RED_LEFT_BACK_X,
				new Rotation2d()
			);

			var forwardPose = new Pose2d(
				alliance == Alliance.Blue
					? SimConstants.STATION_BLUE_LEFT_FORWARD_X
					: SimConstants.STATION_RED_LEFT_FORWARD_X,
				alliance == Alliance.Blue
					? SimConstants.STATION_BLUE_LEFT_FORWARD_X
					: SimConstants.STATION_RED_LEFT_FORWARD_X,
				new Rotation2d()
			);

			shouldGive.set(isPoseOnLine(simRobotPose, backPose, forwardPose));
		});

		return shouldGive.get();
	}

	private static boolean isPoseOnLine(Pose2d pose, Pose2d pointA, Pose2d pointB) {
		var slopeA2B = getSlopeBetweenPoses(pointA, pointB);
		var slopeP2A = getSlopeBetweenPoses(pose, pointA);
		var slopeP2B = getSlopeBetweenPoses(pose, pointB);

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
	 * Drops the coral from the funnel.
	 */
	public void dropSimCoral() {
		if (!simRobotHasCoral()) {
			return;
		}

		var simPose = SimLogging.getInstance().getSimRobotPose();

		SimulatedArena.getInstance()
				.addGamePieceProjectile(new ReefscapeCoralOnFly(
						// Obtain robot position from drive simulation
						simPose.getTranslation(),
						// The scoring mechanism is installed at (0.46, 0) (meters) on the robot
						new Translation2d(
								Units.inchesToMeters(SimConstants.WIDTH_IN)
								/ 2, 0),
						// Obtain robot speed from drive simulation
						SimLogging.getInstance().getSimRobotChassisSpeeds(),
						// Obtain robot facing from drive simulation
						simPose.getRotation(),
						// The height at which the coral is ejected
						Meters.of(
							MechLogging.getInstance().getElevatorStage2().getZ()
							+ Units.inchesToMeters(20)
						),
						// The initial speed of the coral
						MetersPerSecond.of(SimConstants.FUNNEL_OUTTAKE_INIT_SPD_MPS),
						// The coral is ejected vertically downwards
						Degrees.of(SimConstants.FUNNEL_OUTTAKE_ROT_DEG)));
		robotHasCoral = false;
	}

	/**
	 * Determines if the sim robot has a coral loaded in.
	 * @return if the sim robot has a coral loaded in.
	 */
	public boolean simRobotHasCoral() {
		return robotHasCoral;
	}

	/**
	 * Load a coral into the robot.
	 */
	public void loadSimCoral() {
		robotHasCoral = true;
	}


	/**
	 * Get the simulated robot chassis speeds.
	 * @return the simulated robot chassis speeds
	 */
	public ChassisSpeeds getSimRobotChassisSpeeds() {
		return simRobotChassisSpeeds;
	}
}
