// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// Third Party Imports
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
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

// Systems
import frc.robot.systems.FunnelFSMSystem;
import frc.robot.systems.ElevatorFSMSystem;
import frc.robot.systems.DriveFSMSystem;

// Robot Imports
import frc.robot.constants.TunerConstants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends LoggedRobot {
	private TeleopInput input;
	private TunerConstants constants;

	// Systems
	private DriveFSMSystem driveSystem;
	private CommandSwerveDrivetrain swerveDrivetrain;
	private AutoFactory autoFactory;
	private AutoRoutines autoRoutines;
	private AutoChooser autoChooser = new AutoChooser();
	private Command autCommand;
	private FunnelFSMSystem funnelSystem;
	private ElevatorFSMSystem elevatorSystem;


	// Logger
	private PowerDistribution powerLogger;

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

			autoFactory = driveSystem.createAutoFactory();
			autoRoutines = new AutoRoutines(autoFactory, driveSystem);

			autoChooser.addRoutine("testPath", autoRoutines::testAuto);
			SmartDashboard.putData("AUTO CHOOSER", autoChooser);
		}

		if (HardwareMap.isElevatorHardwarePresent()) {
			elevatorSystem = new ElevatorFSMSystem();
		}

		if (HardwareMap.isFunnelHardwarePresent()) {
			funnelSystem = new FunnelFSMSystem();
		}
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		// autoHandler.reset(AutoPath.PATH1);
		autCommand = getAutonomousCommand();

		if (autCommand != null) {
			autCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
		// autoHandler.update();
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
		if (elevatorSystem != null) {
			elevatorSystem.update(input);
		}
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
		Logger.end(); // Stop logging!
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
	}

	@Override
	public void simulationPeriodic() { }

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() { }

	/**
	 * Gets the autonomous command selected by the auto chooser.
	 *
	 * @return the selected autonomous command
	 */
	public Command getAutonomousCommand() {
		return autoChooser.selectedCommand();
	}
}
