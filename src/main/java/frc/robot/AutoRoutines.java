package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.systems.DriveFSMSystem;

public class AutoRoutines {
	private AutoFactory autoFactory;
	private DriveFSMSystem system;

	/**
	 * Constructs an AutoRoutines object with the specified AutoFactory.
	 *
	 * @param factory the AutoFactory to use for creating auto routines
	 */
	public AutoRoutines(AutoFactory factory, DriveFSMSystem system) {
		autoFactory = factory;
		this.system = system;
	}

	/**
	 * Creates and returns a test auto routine.
	 *
	 * @return the test auto routine
	 */
	public AutoRoutine testAuto() {
		final AutoRoutine routine = autoFactory.newRoutine("testPath");
		final AutoTrajectory path1 = routine.trajectory("testPath1");
		final AutoTrajectory path2 = routine.trajectory("testPath2");

		routine.active().onTrue(
			path1.resetOdometry()
			.andThen(path1.cmd())
			.andThen(path2.cmd())
			.andThen(system.brakeCommand())
		);
		return routine;
	}
}
