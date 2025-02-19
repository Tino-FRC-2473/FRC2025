// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// Java Imports
import java.util.HashMap;

// Third Party Imports
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

// WPILib Imports
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.networktables.NetworkTableInstance;

// Systems
import frc.robot.systems.ClimberFSMSystem;
import frc.robot.systems.ElevatorFSMSystem;
import frc.robot.systems.FunnelFSMSystem;
import frc.robot.utils.Elastic;
import frc.robot.systems.DriveFSMSystem;

// Robot Imports
import frc.robot.auto.AutoRoutines;
import frc.robot.logging.MechLogging;
import frc.robot.motors.MotorManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends LoggedRobot {
	private TeleopInput input;

	// Systems
	private DriveFSMSystem driveSystem;
	private AutoRoutines autoRoutines;

	private SendableChooser<String> autoChooser = new SendableChooser<String>();
	private String autoCommand;

	private ElevatorFSMSystem elevatorSystem;
	private FunnelFSMSystem funnelSystem;
	private ClimberFSMSystem climberSystem;

	// Logger
	private PowerDistribution powerLogger;
	private NetworkTableInstance ntInstance;
	private RaspberryPi rpi = new RaspberryPi();
	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");

		Logger.recordMetadata("FRC2025", "Team2473"); // Set a metadata value
		ntInstance = NetworkTableInstance.getDefault();

		if (isReal()) {
			Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
			Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
			powerLogger = new PowerDistribution(1, ModuleType.kRev);
				// Enables power distribution logging
		} else if (isSimulation()) {
			Logger.addDataReceiver(new NT4Publisher());
		} else {
			setUseTiming(false); // Run as fast as possible
			String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope
			Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
		}

		Logger.start(); // Start logging!

		input = new TeleopInput();

		// Instantiate all systems here
		if (Robot.isSimulation() || HardwareMap.isDriveHardwarePresent()) {
			driveSystem = new DriveFSMSystem();
		}

		if (Robot.isSimulation() || HardwareMap.isFunnelHardwarePresent()) {
			funnelSystem = new FunnelFSMSystem();
		}

		if (Robot.isSimulation() || (HardwareMap.isFunnelHardwarePresent()
			&& HardwareMap.isElevatorHardwarePresent())) {
			elevatorSystem = new ElevatorFSMSystem(funnelSystem);
		}

		if (Robot.isSimulation() || HardwareMap.isClimberHardwarePresent()) {
			climberSystem = new ClimberFSMSystem();
		}

		autoRoutines = new AutoRoutines(
			driveSystem, elevatorSystem, funnelSystem
		);

		for (HashMap.Entry<String, Object[]> auto
			: autoRoutines.getAutoPathHandler().getAllAutos().entrySet()) {
			autoChooser.addOption(auto.getKey(), auto.getKey());
		}

		SmartDashboard.putData("AUTO CHOOSER", autoChooser);
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		Elastic.selectTab("Autonomous");
		autoCommand = getAutonomousCommand();

		/* If all available auto systems are true, then it will throw exception. */
		boolean throwException =
			HardwareMap.isCVHardwarePresent()
			&& HardwareMap.isDriveHardwarePresent()
			&& HardwareMap.isElevatorHardwarePresent()
			&& HardwareMap.isFunnelHardwarePresent();

		if (autoCommand != null) {
			Command scheduledCommand = autoRoutines.generateSequentialAutoWorkflow(
				autoRoutines.getAutoPathHandler().getAllAutos().get(autoCommand), throwException
			);

			if (Robot.isSimulation()) {
				driveSystem.getMapleSimDrivetrain().getDriveSimulation()
					.setSimulationWorldPose(autoRoutines.getInitialAutoPose());
			}

			scheduledCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
		if (HardwareMap.isDriveHardwarePresent()) {
			driveSystem.updateAutonomous();
		}
		MotorManager.update();
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		Elastic.selectTab("Teleoperated");
		if (driveSystem != null) {
			driveSystem.reset();
		}
		if (funnelSystem != null) {
			funnelSystem.reset();
		}
		if (climberSystem != null) {
			climberSystem.reset();
		}
		if (elevatorSystem != null) {
			elevatorSystem.reset();
		}
	}

	@Override
	public void teleopPeriodic() {
		if (driveSystem != null) {
			driveSystem.update(input);
		}
		if (funnelSystem != null) {
			funnelSystem.update(input);
		}
		if (climberSystem != null) {
			climberSystem.update(input);
		}
		if (elevatorSystem != null) {
			elevatorSystem.update(input);
		}
		MotorManager.update();
		ntInstance.flush();
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
		if (powerLogger != null) {
			powerLogger.close();
		}
	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void testInit() {
		System.out.println("-------- Test Init --------");
	}

	@Override
	public void testPeriodic() {
		System.out.println(rpi.getAprilTags().toString());
	}

	/* Simulation mode handlers, only used for simulation testing  */
	@Override
	public void simulationInit() {
		System.out.println("-------- Simulation Init --------");
		// don't preform simulated hardware init here, robotInit() still runs during sim
		SimulatedArena.getInstance().resetFieldForAuto();
	}

	@Override
	public void simulationPeriodic() {
		if (HardwareMap.isDriveHardwarePresent()) {
			driveSystem.getMapleSimDrivetrain().update();
		}

		Logger.recordOutput(
			"FieldSimulation/Robot/Primary Elevator Pose",
			MechLogging.getInstance().getPrimaryElevatorPose()
		);

		Logger.recordOutput(
			"FieldSimulation/Robot/Secondary Elevator Pose",
			MechLogging.getInstance().getSecondaryElevatorPose()
		);

		Logger.recordOutput(
			"FieldSimulation/Robot/Climber Pose",
			MechLogging.getInstance().getClimberPose()
		);

		Logger.recordOutput(
			"FieldSimulation/Robot/DriveTrain Pose",
			driveSystem.getMapleSimDrivetrain().getDriveSimulation().getSimulatedDriveTrainPose()
		);

		Logger.recordOutput(
			"FieldSimulation/AlgaePoses",
			SimulatedArena.getInstance().getGamePiecesArrayByType("Algae")
		);

		Logger.recordOutput(
			"FieldSimulation/CoralPoses",
			SimulatedArena.getInstance().getGamePiecesArrayByType("Coral")
		);

		Logger.recordOutput(
			"FieldSimulation/Poses",
			MechLogging.getInstance().getRobotPoses()
		);

	}

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() {
		if (funnelSystem != null) {
			funnelSystem.updateLogging();
		}

		if (elevatorSystem != null) {
			elevatorSystem.updateLogging();
		}

		if (climberSystem != null) {
			climberSystem.updateLogging();
		}
	}

	/**
	 * Gets the autonomous command selected by the auto chooser.
	 *
	 * @return the selected autonomous command
	 */
	public String getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
