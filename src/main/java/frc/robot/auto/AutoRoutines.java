package frc.robot.auto;

import java.io.File;
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
	private AutoPaths autoPaths;

	// Initialize all FSMs (with commands) here
	private DriveFSMSystem driveSystem;
	private ElevatorFSMSystem elevatorSystem;
	private FunnelFSMSystem funnelSystem;
	private ClimberFSMSystem climberSystem;

	// Initialize all paths / commands
	private Map<String, AutoTrajectory> paths = new HashMap<String, AutoTrajectory>();

	private Object[] currentAutoState;

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

		if (HardwareMap.isDriveHardwarePresent()) {
			sysRoutine = driveSystem.createAutoFactory().newRoutine("AutoRoutine");
			generateSysRoutineMap(Filesystem.getDeployDirectory().toString());
		}

		autoPaths = new AutoPaths();
	}

	/**
	 * Creates and returns a auto routine that start with a path.
	 * <br> <br>
	 * For clarification with the throwException parameter, if you set it to FALSE,
	 * it will just not add the command request into the returned SequentialCommandGroup.
	 * This could be helpful for testing series of commands when you are working with partial
	 * systems.
	 * <br> <br>
	 * If all systems are available, it should probably be set to TRUE - in an ideal
	 * environment, no commands should throw this issue if all systems are available. It will be
	 * more useful for debugging in this environment if you set throwException to true.
	 *
	 * @param autoStageSupply string of commands and trajectory names
	 * @param throwException whether to throw an exception at a missing/unknown command or not.
	 * @return the auto routine
	 */
	public SequentialCommandGroup generateSequentialAutoWorkflow(Object[] autoStageSupply,
		boolean throwException) {

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
					if (throwException) {
						throw new IllegalStateException(
							"Unknown trajectory in sequential stage supply."
						);
					} else {
						System.out.println(
							" -------------- \n"
							+ autoStage + " is unavailable."
							+ "Not adding to sequential flow. ");
					}
				}
			} else if (autoStage.getClass().equals(AutoCommands.class)) {
				/* -- Processing commands -- */
				Command processedCommand = initializeCommand((AutoCommands) autoStage);
				if (processedCommand != null) {
					seqInstruction.addCommands(
						processedCommand
						.alongWith(getAutoLogCommand(new String[] {autoStage.toString()}))
					);
				} else {
					if (throwException) {
						throw new IllegalStateException(
							"Unknown command in sequential stage supply."
						);
					} else {
						System.out.println(
							" -------------- \n"
							+ autoStage.toString() + " is unavailable."
							+ " Not adding to sequential flow. ");
					}
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
							if (throwException) {
								throw new IllegalStateException(
									"Unknown trajectory in parallel stage supply."
								);
							} else {
								System.out.println(
									" -------------- \n"
									+ autoParallelStage.toString() + " is unavailable."
									+ " Not adding to parallel staged flow. ");
							}
						}
					/* -- Processing commands -- */
					} else if (autoParallelStage.getClass().equals(AutoCommands.class)) {
						Command processedCommand = initializeCommand(
							(AutoCommands) autoParallelStage
						);

						if (processedCommand != null) {
							parallelQueue.addCommands(processedCommand);
						} else {
							if (throwException) {
								throw new IllegalStateException(
									"Unknown command in parallel stage supply."
								);
							} else {
								System.out.println(
									" -------------- \n"
									+ autoParallelStage.toString() + " is unavailable."
									+ " Not adding to parallel stage flow. ");
							}
						}
					}
				}

				parallelQueue.addCommands(getAutoLogCommand((Object[]) autoStage));
				seqInstruction.addCommands(parallelQueue);

			} else {
				if (throwException) {
					throw new IllegalStateException(
						"Unknown parameter in stage supply."
					);
				} else {
					System.out.println(
						" -------------- \n"
						+ autoStage.toString() + " is an unkown parameter."
						+ " Not adding to sequential flow. ");
				}
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


	private Command initializeCommand(AutoCommands commandEntry) {

		Command returnInitCommand = null;

		if (HardwareMap.isDriveHardwarePresent()) {
			returnInitCommand = (returnInitCommand == null)
				? checkDriveCommands(commandEntry) : returnInitCommand;

			if (HardwareMap.isCVHardwarePresent()) {
				returnInitCommand = (returnInitCommand == null)
					? checkAlignmentCommands(commandEntry) : returnInitCommand;
			}
		}

		if (HardwareMap.isElevatorHardwarePresent()) {
			returnInitCommand = (returnInitCommand == null)
				? checkElevatorCommands(commandEntry) : returnInitCommand;
		}

		if (HardwareMap.isFunnelHardwarePresent()) {
			returnInitCommand = (returnInitCommand == null)
				? checkFunnelCommands(commandEntry) : returnInitCommand;
		}

		return returnInitCommand;
	}

	/**
	 * Get an AutoPaths instance to return all declared paths in AutoPaths.java.
	 * @return AutoPaths instance
	 */
	public AutoPaths getAutoPathHandler() {
		return autoPaths;
	}

	private Command checkDriveCommands(AutoCommands commandEntry) {
		/* ---- All Drive Commands ---- */
		switch (commandEntry) {
			case DRIVE_BRAKE_CMD:
				return driveSystem.brakeCommand();
			case DRIVE_ROBOT_LEFT_RELATIVE_OFFSET_TIMED_CMD:
				return driveSystem.driveRobotRelativeOffset(
					AutoConstants.REEF_OFFSET_X_AUTO_SPEED,
					AutoConstants.REEF_OFFSET_Y_AUTO_SPEED,
					AutoConstants.TIME_DRIVING_OFFSET);
			case DRIVE_ROBOT_RIGHT_RELATIVE_OFFSET_TIMED_CMD:
				return driveSystem.driveRobotRelativeOffset(
					-AutoConstants.REEF_OFFSET_X_AUTO_SPEED,
					AutoConstants.REEF_OFFSET_Y_AUTO_SPEED,
					AutoConstants.TIME_DRIVING_OFFSET);
			default:
				return null;
		}
	}

	private Command checkAlignmentCommands(AutoCommands commandEntry) {
		/* ---- All Red AprilTag Alignment Commands ---- */
		switch (commandEntry) {
			case R_ALIGN_REEF2_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_2_TAG_ID
				);
			case R_ALIGN_REEF3_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_3_TAG_ID
				);
			case R_ALIGN_REEF5_TAG_CMD:
				return driveSystem.alignToTagCommand(
						AutoConstants.R_REEF_5_TAG_ID
				);
			case R_ALIGN_REEF6_TAG_CMD:
				return driveSystem.alignToTagCommand(
						AutoConstants.R_REEF_6_TAG_ID
				);
			case R_ALIGN_STATION_L_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.RED_L_STATION_ID
				);
			case R_ALIGN_STATION_R_TAG_CMD:
				return driveSystem.alignToTagCommand(
						AutoConstants.RED_R_STATION_ID
				);

			/* ---- All Blue AprilTag Alignment Commands ---- */
			case B_ALIGN_REEF2_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_2_TAG_ID
				);
			case B_ALIGN_REEF3_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_3_TAG_ID
				);
			case B_ALIGN_REEF5_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_5_TAG_ID
				);
			case B_ALIGN_REEF6_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_6_TAG_ID
				);
			case B_ALIGN_STATION_L_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.BLUE_L_STATION_ID
				);
			case B_ALIGN_STATION_R_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.BLUE_R_STATION_ID
				);
			default:
				return null;
		}
	}

	private Command checkElevatorCommands(AutoCommands commandEntry) {
		switch (commandEntry) {
			case ELEVATOR_GROUND_CMD:
				return elevatorSystem.elevatorGroundCommand();
			case ELEVATOR_STATION_CMD:
				return elevatorSystem.elevatorStationCommand();
			case ELEVATOR_L4_CMD:
				return elevatorSystem.elevatorL4Command();
			default:
				return null;
		}
	}

	private Command checkFunnelCommands(AutoCommands commandEntry) {
		switch (commandEntry) {
			case FUNNEL_OPEN_CMD:
				return funnelSystem.openFunnelCommand();
			case FUNNEL_CLOSE_CMD:
				return funnelSystem.closeFunnelCommand();
			default:
				return null;
		}
	}
}
