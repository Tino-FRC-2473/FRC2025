// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

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
// Systems
import frc.robot.systems.ClimberFSMSystem;
import frc.robot.systems.ElevatorFSMSystem;
import frc.robot.systems.FunnelFSMSystem;
import frc.robot.systems.DriveFSMSystem;

// Robot Imports
import frc.robot.auto.AutoRoutines;
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
	private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
	private Command autCommand;
	private FunnelFSMSystem funnelSystem;
	private ClimberFSMSystem climberSystem;
	private ElevatorFSMSystem elevatorSystem;

	// Logger
	private PowerDistribution powerLogger;

	private static final Object[] PATH_1 = new Object[] {
		"S1_R2"
	};

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");

		Logger.recordMetadata("FRC2025", "Team2473"); // Set a metadata value

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
		if (HardwareMap.isDriveHardwarePresent()) {
			driveSystem = new DriveFSMSystem();

			autoRoutines = new AutoRoutines(driveSystem);

			autoChooser.addOption("Path 1",
				autoRoutines.generateSequentialAutoWorkflow(PATH_1).cmd());
			SmartDashboard.putData("AUTO CHOOSER", autoChooser);
		}

		if (HardwareMap.isFunnelHardwarePresent()) {
			funnelSystem = new FunnelFSMSystem();
		}

		if (HardwareMap.isElevatorHardwarePresent()) {
			elevatorSystem = new ElevatorFSMSystem();
		}

		if (HardwareMap.isClimberHardwarePresent()) {
			climberSystem = new ClimberFSMSystem();
		}
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		autCommand = getAutonomousCommand();

		if (autCommand != null) {
			autCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
		driveSystem.updateAutonomous();
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
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
		driveSystem.getMapleSimDrivetrain().update();

		Logger.recordOutput(
			"FieldSimulation/SimulatedPose",
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

	}

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() { }

	/**
	 * Gets the autonomous command selected by the auto chooser.
	 *
	 * @return the selected autonomous command
	 */
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
