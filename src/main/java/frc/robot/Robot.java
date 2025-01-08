// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
// Third Party Imports
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

// WPILib Imports


// Systems
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.FunnelFSMSystem;
import frc.robot.systems.ElevatorFSMSystem;
import frc.robot.systems.AutoHandlerSystem;
import frc.robot.systems.AutoHandlerSystem.AutoPath;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends LoggedRobot {
	private TeleopInput input;

	// Systems
	private DriveFSMSystem driveSystem;
	private FunnelFSMSystem funnelSystem;
	private ElevatorFSMSystem elevatorSystem;

	private AutoHandlerSystem autoHandler;

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
			new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
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
		}

		if (HardwareMap.isMech1HardwarePresent()) {
			funnelSystem = new FunnelFSMSystem();
		}

		if (HardwareMap.isMech2HardwarePresent()) {
			elevatorSystem = new ElevatorFSMSystem();
		}
		autoHandler = new AutoHandlerSystem(driveSystem, funnelSystem, elevatorSystem);
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		autoHandler.reset(AutoPath.PATH1);
	}

	@Override
	public void autonomousPeriodic() {
		autoHandler.update();
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		driveSystem.reset();
		funnelSystem.reset();
		elevatorSystem.reset();
	}

	@Override
	public void teleopPeriodic() {
		driveSystem.update(input);
		funnelSystem.update(input);
		elevatorSystem.update(input);
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
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
	}

	@Override
	public void simulationPeriodic() { }

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() { }
}
