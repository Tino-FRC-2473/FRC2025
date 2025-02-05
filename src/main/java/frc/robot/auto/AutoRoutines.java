package frc.robot.auto;

import java.io.File;
import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HardwareMap;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.AutoConstants.AutoCommands;
import frc.robot.systems.ClimberFSMSystem;
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.ElevatorFSMSystem;
import frc.robot.systems.FunnelFSMSystem;

public class AutoRoutines {

	// Auto sys instance -- used to convert choreo trajectories into schedulable commands.
	private AutoRoutine sysRoutine;

	// Initialize all FSMs (with commands) here
	private DriveFSMSystem driveSystem;
	private ElevatorFSMSystem elevatorSystem;
	private FunnelFSMSystem funnelSystem;
	private ClimberFSMSystem climberSystem;

	// Initialize all paths / commands
	private Map<String, AutoTrajectory> paths = new HashMap<String, AutoTrajectory>();
	private Map<AutoCommands, Command> commands = new HashMap<AutoCommands, Command>();

	private Object[] currentAutoState;

	/* ----------------------------------- ALL PATHS ------------------------------------- */
	private Object[] bPathOne = new Object[] {
		funnelSystem.closeFunnelCommand(),
		new Object[] {
			driveSystem.alignToTagCommand(AutoConstants.B_REEF_1_TAG_ID),
			elevatorSystem.elevatorStationCommand()
		},
		new Object[] {
			driveSystem.driveRobotRightRelativeOffset(),
			elevatorSystem.elevatorL4Command()
		},
		funnelSystem.openFunnelCommand(),
		new Object[] {
			elevatorSystem.elevatorStationCommand(),
			"R1_StationL"
		},
		funnelSystem.closeFunnelCommand(),
		driveSystem.alignToTagCommand(AutoConstants.B_REEF_3_TAG_ID),
		new Object[] {
			driveSystem.driveRobotLeftRelativeOffset(),
			elevatorSystem.elevatorL4Command()
		},
		funnelSystem.openFunnelCommand(),
		driveSystem.brakeCommand()
	};

	/* --------------------------------------------------------------------------------------- */

	/**
	 * Constructs an AutoRoutines object.
	 * @param driveFSMSystem
	 * @param elevatorFSMSystem
	 * @param funnelFSMSystem
	 * @param climberFSMSystem
	 * */
	public AutoRoutines(DriveFSMSystem driveFSMSystem, ElevatorFSMSystem elevatorFSMSystem,
		FunnelFSMSystem funnelFSMSystem, ClimberFSMSystem climberFSMSystem) {

		// Assign systems
		driveSystem = driveFSMSystem;
		elevatorSystem = elevatorFSMSystem;
		funnelSystem = funnelFSMSystem;
		climberSystem = climberFSMSystem;

		// Set up commands for each system
		initialize();
	}

	/**
	 * Creates and returns a auto routine that start with a path.
	 * @param autoStageSupply string of commands and trajectory names
	 * @return the auto routine
	 */
	public SequentialCommandGroup generateSequentialAutoWorkflow(Object[] autoStageSupply) {

		SequentialCommandGroup seqInstruction = new SequentialCommandGroup();

		for (int i = 0; i < autoStageSupply.length; i++) {
			var autoStage = autoStageSupply[i];

			if (autoStage.getClass().equals(String.class)) {
				/* -- Processing drive trajs -- */
				if (HardwareMap.isDriveHardwarePresent() && paths.containsKey(autoStage)) {
					AutoTrajectory traj = paths.get(autoStage);
					if (i == 0) {
						seqInstruction.addCommands(traj.resetOdometry());
					}

					seqInstruction.addCommands(
						traj.cmd()
						.alongWith(getAutoLogCommand(new String[] {(String) autoStage}))
					);
				} else {
					throw new IllegalStateException(
						"Unknown trajectory in sequential stage supply."
					);
				}
			} else if (autoStage.getClass().equals(Command.class)) {
				/* -- Processing mech commands -- */
				seqInstruction.addCommands(
					((Command) autoStage)
					.alongWith(getAutoLogCommand(
						new String[] {autoStage.getClass().getSimpleName()}
					))
				);
			/* --- Processing parallel auto commands --- */
			} else if (autoStage.getClass().equals(Object[].class)) {

				ParallelCommandGroup parallelQueue = new ParallelCommandGroup();
				String[] loggingString = new String[((Object[]) autoStage).length];
				int t = 0;

				for (Object autoParallelStage: (Object[]) autoStage) {

					/* -- Processing drive trajs -- */
					if (autoParallelStage.getClass().equals(String.class)
						&& driveSystem != null) {
						if (paths.containsKey(autoParallelStage)) {
							AutoTrajectory traj = paths.get(autoParallelStage);
							if (i == 0) {
								parallelQueue.addCommands(traj.resetOdometry());
							}

							parallelQueue.addCommands(traj.cmd());
							loggingString[t++] = (String) autoParallelStage;
						} else {
							throw new IllegalStateException(
								"Unknown trajectory in parallel stage supply."
							);
						}
					/* -- Processing mech commands -- */
					} else if (autoParallelStage.getClass().equals(Command.class)) {
						parallelQueue.addCommands(commands.get(autoParallelStage));
						loggingString[t++] = autoParallelStage.getClass().getSimpleName();
					}
				}

				parallelQueue.addCommands(getAutoLogCommand(loggingString));
				seqInstruction.addCommands(parallelQueue);

			} else {
				throw new IllegalStateException(
					"Unknown parameter in stage supply."
				);
			}
		}

		if (HardwareMap.isDriveHardwarePresent()) {
			seqInstruction.addCommands(driveSystem.brakeCommand());
		}

		seqInstruction.schedule();

		return seqInstruction;
	}

	private void generateSysRoutineMap(String deployFolder) {
		File deployDir = new File(deployFolder + "/choreo");

		for (File choreoFile : deployDir.listFiles()) {
			if (choreoFile.getName().endsWith(".traj")) {
				paths.put(choreoFile.getName()
					.replace(".traj", ""),
					sysRoutine.trajectory(choreoFile.getName()));
			}
		}
	}

	private Command getAutoLogCommand(Object[] cAutoState) {
		class AutoLogCommand extends Command {

			@Override
			public boolean isFinished() {
				currentAutoState = cAutoState;
				SmartDashboard.putString("Auto State", Arrays.toString(currentAutoState));
				return true;
			}
		}

		return new AutoLogCommand();
	}


	private void initialize() {

		if (HardwareMap.isDriveHardwarePresent()) {

			sysRoutine = driveSystem.createAutoFactory().newRoutine("AutoRoutine");
			generateSysRoutineMap(Filesystem.getDeployDirectory().toString());

		}
	}

	/**
	 * Get all auto object arrays declared in the AutoRoutines file.
	 * @return array of all Object[] autos
	 */
	public HashMap<String, Object[]> getAllAutos() {
		HashMap<String, Object[]> allObjArrays = new HashMap<String, Object[]>();
		Field[] allFields = this.getClass().getDeclaredFields();

		for (Field f: allFields) {
			if (f.getType().equals(Object[].class)) {
				try {
					f.setAccessible(true);
					Object[] array = (Object[]) f.get(this);
					allObjArrays.put(f.getName(), array);
				} catch (IllegalAccessException e) {
					e.printStackTrace();
				}
			}
		}

		return allObjArrays;
	}
}
