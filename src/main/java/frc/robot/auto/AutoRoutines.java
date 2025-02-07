package frc.robot.auto;

import java.io.File;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HardwareMap;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.AutoConstants.AutoCommands;
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

	// Initialize all paths / commands
	private Map<String, AutoTrajectory> paths = new HashMap<String, AutoTrajectory>();
	private Map<AutoCommands, Command> commands = new HashMap<AutoCommands, Command>();

	private Object[] currentAutoState;

	/**
	 * Constructs an AutoRoutines object.
	 * @param driveFSMSystem
	 * @param elevatorFSMSystem
	 * @param funnelFSMSystem
	 * */
	public AutoRoutines(DriveFSMSystem driveFSMSystem, ElevatorFSMSystem elevatorFSMSystem,
		FunnelFSMSystem funnelFSMSystem) {

		// Assign systems
		driveSystem = driveFSMSystem;
		elevatorSystem = elevatorFSMSystem;
		funnelSystem = funnelFSMSystem;

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
					if (autoParallelStage.getClass().equals(String.class)
						&& driveSystem != null) {
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

	// Initialize commands for each system
	private void initialize() {
		// Set up commands
		if (driveSystem != null) {
			sysRoutine = driveSystem.createAutoFactory().newRoutine("AutoRoutine");

			setUpDriveCommands();

			generateSysRoutineMap("src/main/deploy");
		}
		if (elevatorSystem != null) {
			setUpElevatorCommands();
		}
		if (funnelSystem != null) {
			setUpFunnelCommands();
		}
	}

	private void setUpDriveCommands() {
		/* ---- All Red AprilTag Alignment Commands ---- */
		commands.put(AutoCommands.R_ALIGN_REEF2_L_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_2_TAG_ID,
					AutoConstants.REEF_X_L_TAG_OFFSET,
					AutoConstants.REEF_Y_L_TAG_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_REEF2_R_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_2_TAG_ID,
					AutoConstants.REEF_X_R_TAG_OFFSET,
					AutoConstants.REEF_Y_R_TAG_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_REEF3_L_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_3_TAG_ID,
					AutoConstants.REEF_X_L_TAG_OFFSET,
					AutoConstants.REEF_Y_L_TAG_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_REEF3_R_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_3_TAG_ID,
					AutoConstants.REEF_X_R_TAG_OFFSET,
					AutoConstants.REEF_Y_R_TAG_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_REEF5_L_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_5_TAG_ID,
					AutoConstants.REEF_X_L_TAG_OFFSET,
					AutoConstants.REEF_Y_L_TAG_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_REEF5_R_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_5_TAG_ID,
					AutoConstants.REEF_X_R_TAG_OFFSET,
					AutoConstants.REEF_Y_R_TAG_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_REEF6_L_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_5_TAG_ID,
					AutoConstants.REEF_X_L_TAG_OFFSET,
					AutoConstants.REEF_Y_L_TAG_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_REEF6_R_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_5_TAG_ID,
					AutoConstants.REEF_X_R_TAG_OFFSET,
					AutoConstants.REEF_Y_R_TAG_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_STATION_L_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.RED_L_STATION_ID,
					AutoConstants.SOURCE_X_OFFSET,
					AutoConstants.SOURCE_Y_OFFSET)
		);
		commands.put(AutoCommands.R_ALIGN_STATION_R_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.RED_R_STATION_ID,
					AutoConstants.SOURCE_X_OFFSET,
					AutoConstants.SOURCE_Y_OFFSET)
		);

		/* ---- All Blue AprilTag Alignment Commands ---- */
		commands.put(AutoCommands.B_ALIGN_REEF2_L_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_2_TAG_ID,
					AutoConstants.REEF_X_L_TAG_OFFSET,
					AutoConstants.REEF_Y_L_TAG_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_REEF2_R_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_2_TAG_ID,
					AutoConstants.REEF_X_R_TAG_OFFSET,
					AutoConstants.REEF_Y_R_TAG_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_REEF3_L_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_3_TAG_ID,
					AutoConstants.REEF_X_L_TAG_OFFSET,
					AutoConstants.REEF_Y_L_TAG_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_REEF3_R_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_3_TAG_ID,
					AutoConstants.REEF_X_R_TAG_OFFSET,
					AutoConstants.REEF_Y_R_TAG_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_REEF5_L_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_5_TAG_ID,
					AutoConstants.REEF_X_L_TAG_OFFSET,
					AutoConstants.REEF_Y_L_TAG_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_REEF5_R_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_5_TAG_ID,
					AutoConstants.REEF_X_R_TAG_OFFSET,
					AutoConstants.REEF_Y_R_TAG_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_REEF6_L_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_5_TAG_ID,
					AutoConstants.REEF_X_L_TAG_OFFSET,
					AutoConstants.REEF_Y_L_TAG_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_REEF6_R_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_5_TAG_ID,
					AutoConstants.REEF_X_R_TAG_OFFSET,
					AutoConstants.REEF_Y_R_TAG_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_STATION_L_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.BLUE_L_STATION_ID,
					AutoConstants.SOURCE_X_OFFSET,
					AutoConstants.SOURCE_Y_OFFSET)
		);
		commands.put(AutoCommands.B_ALIGN_STATION_R_TAG_CMD,
			driveSystem.alignToTagCommand(
					AutoConstants.BLUE_R_STATION_ID,
					AutoConstants.SOURCE_X_OFFSET,
					AutoConstants.SOURCE_Y_OFFSET)
		);

		/* ---- All Drive Commands ---- */
		commands.put(AutoCommands.DRIVE_BRAKE_CMD,
			driveSystem.brakeCommand()
		);
	}

	private void setUpElevatorCommands() {
		commands.put(AutoCommands.ELEVATOR_GROUND_CMD,
			elevatorSystem.elevatorGroundCommand()
		);
		commands.put(AutoCommands.ELEVATOR_L2_CMD,
			elevatorSystem.elevatorL2Command()
		);
		commands.put(AutoCommands.ELEVATOR_L3_CMD,
			elevatorSystem.elevatorL3Command()
		);
		commands.put(AutoCommands.ELEVATOR_L4_CMD,
			elevatorSystem.elevatorL4Command()
		);
		commands.put(AutoCommands.WAIT,
			elevatorSystem.waitCommand()
		);
	}

	private void setUpFunnelCommands() {
		commands.put(AutoCommands.FUNNEL_OPEN_CMD,
			funnelSystem.openFunnelCommand()
		);
		commands.put(AutoCommands.FUNNEL_CLOSE_CMD,
			funnelSystem.closeFunnelCommand()
		);
	}
}
