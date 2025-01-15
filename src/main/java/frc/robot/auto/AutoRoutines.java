package frc.robot.auto;

import java.io.File;
import java.util.HashMap;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.AutoConstants.AutoCommands;
import frc.robot.systems.DriveFSMSystem;

public class AutoRoutines {
	private AutoFactory autoFactory;

	// Initialize all FSMs (with commands) here
	private DriveFSMSystem driveSystem;


	// Initialize all paths
	private AutoRoutine sysRoutine;

	private HashMap<String, AutoTrajectory> paths;
	private HashMap<AutoCommands, Command> commands;
	private Object[] currentAutoState;

	/**
	 * Constructs an AutoRoutines object with the specified AutoFactory.
	 * @param system1
	 * */
	public AutoRoutines(DriveFSMSystem system1) {
		driveSystem = system1;

		autoFactory = driveSystem.createAutoFactory();
		sysRoutine = autoFactory.newRoutine("AutoRoutine");

		paths = new HashMap<String, AutoTrajectory>();
		commands = new HashMap<AutoCommands, Command>();

		setupCommands();
		generateSysRoutineMap("src/main/deploy");
	}

	/**
	 * Creates and returns a auto routine that start with a path.
	 * @param autoStageSupply string of commands and trajectory names
	 * @return the auto routine
	 */
	public AutoRoutine generateSequentialAutoWorkflow(Object[] autoStageSupply) {

		SequentialCommandGroup seqInstruction = new SequentialCommandGroup();

		for (int i = 0; i < autoStageSupply.length; i++) {
			var autoStage = autoStageSupply[i];

			if (autoStage.getClass().equals(String.class)) {
				/* -- Processing drive trajs -- */
				if (paths.containsKey(autoStage)) {
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
			} else if (autoStage.getClass().equals(AutoCommands.class)) {
				/* -- Processing commands -- */
				if (commands.containsKey(autoStage)) {
					seqInstruction.addCommands(
						commands.get(autoStage)
						.alongWith(getAutoLogCommand(new String[] {autoStage.toString()}))
					);
				} else {
					throw new IllegalStateException(
						"Unknown command in sequential stage supply."
					);
				}
			} else if (autoStage.getClass().equals(Object[].class)) {

				ParallelCommandGroup parallelQueue = new ParallelCommandGroup();

				for (Object autoParallelStage: (Object[]) autoStage) {

					/* -- Processing drive trajs -- */
					if (autoParallelStage.getClass().equals(String.class)) {
						if (paths.containsKey(autoParallelStage)) {
							AutoTrajectory traj = paths.get(autoParallelStage);
							if (i == 0) {
								parallelQueue.addCommands(traj.resetOdometry());
							}

							parallelQueue.addCommands(traj.cmd());
						} else {
							throw new IllegalStateException(
								"Unknown trajectory in parallel stage supply."
							);
						}
					/* -- Processing commands -- */
					} else if (autoParallelStage.getClass().equals(AutoCommands.class)) {
						if (commands.containsKey(autoParallelStage)) {
							parallelQueue.addCommands(commands.get(autoParallelStage));
						} else {
							throw new IllegalStateException(
								"Unknown command in parallel stage supply."
							);
						}
					}
				}

				parallelQueue.addCommands(getAutoLogCommand((Object[]) autoStage));
				seqInstruction.addCommands(parallelQueue);

			} else {
				throw new IllegalStateException(
					"Unknown parameter in stage supply."
				);
			}
		}

		sysRoutine.active().onTrue(
			seqInstruction
			.andThen(driveSystem.brakeCommand())
		);

		return sysRoutine;
	}

	// This function works
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
				SmartDashboard.putString("Auto State", currentAutoState.toString());
				return true;
			}
		}

		return new AutoLogCommand();
	}

	// This function works
	private void setupCommands() {
		/* ---- All Red AprilTag Alignment Commands ---- */

		commands.put(AutoCommands.R_ALIGN_REEF2_L_TAG_CMD,
			driveSystem.alignToReefTagCommand(
					AutoConstants.R_REEF_2_TAG_ID,
					AutoConstants.REEF_X_L_TAG_OFFSET,
					AutoConstants.REEF_Y_L_TAG_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_REEF2_R_TAG_CMD,
			driveSystem.alignToReefTagCommand(
					AutoConstants.R_REEF_2_TAG_ID,
					AutoConstants.REEF_X_R_TAG_OFFSET,
					AutoConstants.REEF_Y_R_TAG_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_REEF3_L_TAG_CMD,
			driveSystem.alignToReefTagCommand(
					AutoConstants.R_REEF_3_TAG_ID,
					AutoConstants.REEF_X_L_TAG_OFFSET,
					AutoConstants.REEF_Y_L_TAG_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_REEF3_R_TAG_CMD,
			driveSystem.alignToReefTagCommand(
					AutoConstants.R_REEF_3_TAG_ID,
					AutoConstants.REEF_X_R_TAG_OFFSET,
					AutoConstants.REEF_Y_R_TAG_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_REEF5_L_TAG_CMD,
			driveSystem.alignToReefTagCommand(
					AutoConstants.R_REEF_5_TAG_ID,
					AutoConstants.REEF_X_L_TAG_OFFSET,
					AutoConstants.REEF_Y_L_TAG_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_REEF5_R_TAG_CMD,
			driveSystem.alignToReefTagCommand(
					AutoConstants.R_REEF_5_TAG_ID,
					AutoConstants.REEF_X_R_TAG_OFFSET,
					AutoConstants.REEF_Y_R_TAG_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_REEF6_L_TAG_CMD,
			driveSystem.alignToReefTagCommand(
					AutoConstants.R_REEF_5_TAG_ID,
					AutoConstants.REEF_X_L_TAG_OFFSET,
					AutoConstants.REEF_Y_L_TAG_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_REEF6_R_TAG_CMD,
			driveSystem.alignToReefTagCommand(
					AutoConstants.R_REEF_5_TAG_ID,
					AutoConstants.REEF_X_R_TAG_OFFSET,
					AutoConstants.REEF_Y_R_TAG_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_STATION_L_TAG_CMD,
			driveSystem.alignToSourceTagCommand(
					AutoConstants.RED_L_STATION_ID,
					AutoConstants.SOURCE_X_OFFSET,
					AutoConstants.SOURCE_Y_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_STATION_R_TAG_CMD,
			driveSystem.alignToSourceTagCommand(
					AutoConstants.RED_R_STATION_ID,
					AutoConstants.SOURCE_X_OFFSET,
					AutoConstants.SOURCE_Y_OFFSET)
		);

		/* ---- All Blue AprilTag Alignment Commands ---- */

		commands.put(AutoCommands.B_ALIGN_REEF2_L_TAG_CMD,
			driveSystem.alignToReefTagCommand(
					AutoConstants.B_REEF_2_TAG_ID,
					AutoConstants.REEF_X_L_TAG_OFFSET,
					AutoConstants.REEF_Y_L_TAG_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_REEF2_R_TAG_CMD,
			driveSystem.alignToReefTagCommand(
					AutoConstants.B_REEF_2_TAG_ID,
					AutoConstants.REEF_X_R_TAG_OFFSET,
					AutoConstants.REEF_Y_R_TAG_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_REEF3_L_TAG_CMD,
			driveSystem.alignToReefTagCommand(
					AutoConstants.B_REEF_3_TAG_ID,
					AutoConstants.REEF_X_L_TAG_OFFSET,
					AutoConstants.REEF_Y_L_TAG_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_REEF3_R_TAG_CMD,
			driveSystem.alignToReefTagCommand(
					AutoConstants.B_REEF_3_TAG_ID,
					AutoConstants.REEF_X_R_TAG_OFFSET,
					AutoConstants.REEF_Y_R_TAG_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_REEF5_L_TAG_CMD,
			driveSystem.alignToReefTagCommand(
					AutoConstants.B_REEF_5_TAG_ID,
					AutoConstants.REEF_X_L_TAG_OFFSET,
					AutoConstants.REEF_Y_L_TAG_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_REEF5_R_TAG_CMD,
			driveSystem.alignToReefTagCommand(
					AutoConstants.B_REEF_5_TAG_ID,
					AutoConstants.REEF_X_R_TAG_OFFSET,
					AutoConstants.REEF_Y_R_TAG_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_REEF6_L_TAG_CMD,
			driveSystem.alignToReefTagCommand(
					AutoConstants.B_REEF_5_TAG_ID,
					AutoConstants.REEF_X_L_TAG_OFFSET,
					AutoConstants.REEF_Y_L_TAG_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_REEF6_R_TAG_CMD,
			driveSystem.alignToReefTagCommand(
					AutoConstants.B_REEF_5_TAG_ID,
					AutoConstants.REEF_X_R_TAG_OFFSET,
					AutoConstants.REEF_Y_R_TAG_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_STATION_L_TAG_CMD,
			driveSystem.alignToSourceTagCommand(
					AutoConstants.BLUE_L_STATION_ID,
					AutoConstants.SOURCE_X_OFFSET,
					AutoConstants.SOURCE_Y_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_STATION_R_TAG_CMD,
			driveSystem.alignToSourceTagCommand(
					AutoConstants.BLUE_R_STATION_ID,
					AutoConstants.SOURCE_X_OFFSET,
					AutoConstants.SOURCE_Y_OFFSET)
		);

		/* ---- All Drive Commands ---- */
		commands.put(AutoCommands.DRIVE_BRAKE_CMD,
			driveSystem.brakeCommand()
		);

		/* ---- All Elevator Commands ---- */

		/* ---- All Intake Commands ---- */

	}
}
