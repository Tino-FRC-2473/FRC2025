package frc.robot.auto;

import java.io.File;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HardwareMap;
import frc.robot.Robot;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.AutoConstants.AutoCommands;
import frc.robot.constants.SimConstants;
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.ElevatorFSMSystem;
import frc.robot.systems.FunnelFSMSystem;

public class AutoRoutines {

	// Auto sys instance -- used to convert choreo trajectories into schedulable commands.
	private AutoRoutine sysRoutine;
	private AutoPaths autoPaths;

	// for sim purposes
	private Pose2d initPose = new Pose2d();

	// Initialize all FSMs (with commands) here
	private DriveFSMSystem driveSystem;
	private ElevatorFSMSystem elevatorSystem;
	private FunnelFSMSystem funnelSystem;

	// Initialize all paths / commands
	private Map<String, AutoTrajectory> paths = new HashMap<String, AutoTrajectory>();

	/* ------------------------ ALL COMMANDS AND PRESETS ------------------------- */
	/* ----- this is the only part you should ever have to edit in this file ----- */

	private Command checkDriveCommands(AutoCommands commandEntry) {
		/* ---- All Drive Commands ---- */
		switch (commandEntry) {
			case DRIVE_BRAKE_CMD:
				return driveSystem.brakeCommand();
			default:
				return null;
		}
	}

	private Command checkAlignmentCommands(AutoCommands commandEntry) {
		/* ---- All Red AprilTag Alignment Commands ---- */
		switch (commandEntry) {
			case R_ALIGN_REEF1_L_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_1_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_L_TAG_OFFSET
				);
			case R_ALIGN_REEF1_R_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_1_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_L_TAG_OFFSET
				);
			case R_ALIGN_REEF2_L_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_2_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_L_TAG_OFFSET
				);
			case R_ALIGN_REEF2_R_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_2_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_R_TAG_OFFSET
				);
			case R_ALIGN_REEF3_L_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_3_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_L_TAG_OFFSET
				);
			case R_ALIGN_REEF3_R_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_3_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_R_TAG_OFFSET
				);
			case R_ALIGN_REEF4_L_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_4_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_L_TAG_OFFSET
				);
			case R_ALIGN_REEF4_R_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_4_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_R_TAG_OFFSET
				);
			case R_ALIGN_REEF5_L_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_5_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_L_TAG_OFFSET
				);
			case R_ALIGN_REEF5_R_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_5_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_R_TAG_OFFSET
				);
			case R_ALIGN_REEF6_L_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_6_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_L_TAG_OFFSET
				);
			case R_ALIGN_REEF6_R_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.R_REEF_6_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_R_TAG_OFFSET
				);
			case R_ALIGN_STATION_L_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.RED_L_STATION_ID,
					AutoConstants.SOURCE_X_OFFSET, AutoConstants.SOURCE_Y_OFFSET
				);
			case R_ALIGN_STATION_R_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.RED_R_STATION_ID,
					AutoConstants.SOURCE_X_OFFSET, AutoConstants.SOURCE_Y_OFFSET
				);
			/* ---- All Blue AprilTag Alignment Commands ---- */
			case B_ALIGN_REEF1_L_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_1_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_L_TAG_OFFSET
				);
			case B_ALIGN_REEF1_R_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_1_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_R_TAG_OFFSET
				);
			case B_ALIGN_REEF2_L_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_2_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_L_TAG_OFFSET
				);
			case B_ALIGN_REEF2_R_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_2_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_R_TAG_OFFSET
				);
			case B_ALIGN_REEF3_L_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_3_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_L_TAG_OFFSET
				);
			case B_ALIGN_REEF3_R_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_3_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_R_TAG_OFFSET
				);
			case B_ALIGN_REEF4_L_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_4_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_L_TAG_OFFSET
				);
			case B_ALIGN_REEF4_R_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_4_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_R_TAG_OFFSET
				);
			case B_ALIGN_REEF5_L_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_5_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_L_TAG_OFFSET
				);
			case B_ALIGN_REEF5_R_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_5_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_R_TAG_OFFSET
				);
			case B_ALIGN_REEF6_L_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_6_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_L_TAG_OFFSET
				);
			case B_ALIGN_REEF6_R_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.B_REEF_6_TAG_ID,
					AutoConstants.REEF_X_TAG_OFFSET, AutoConstants.REEF_Y_R_TAG_OFFSET
				);
			case B_ALIGN_STATION_L_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.BLUE_L_STATION_ID,
					AutoConstants.SOURCE_X_OFFSET, AutoConstants.SOURCE_Y_OFFSET
				);
			case B_ALIGN_STATION_R_TAG_CMD:
				return driveSystem.alignToTagCommand(
					AutoConstants.BLUE_R_STATION_ID,
					AutoConstants.SOURCE_X_OFFSET, AutoConstants.SOURCE_Y_OFFSET
				);
			default:
				return null;
		}
	}

	private Command checkElevatorCommands(AutoCommands commandEntry) {
		switch (commandEntry) {
			case ELEVATOR_GROUND_CMD:
				return elevatorSystem.elevatorGroundCommand();
			case ELEVATOR_L2_CMD:
				return elevatorSystem.elevatorL2Command();
			case ELEVATOR_L3_CMD:
				return elevatorSystem.elevatorL3Command();
			case ELEVATOR_L4_CMD:
				return elevatorSystem.elevatorL4Command();
			case ELEVATOR_WAIT:
				return elevatorSystem.waitCommand();
			default:
				return null;
		}
	}

	private Command checkFunnelCommands(AutoCommands commandEntry) {
		System.out.println("REACHED FUNNEL PARSE");
		switch (commandEntry) {
			case INTAKE_CORAL_CMD:
				return funnelSystem.intakeCoralCommand();
			case OUTTAKE_CORAL_CMD:
				return funnelSystem.outtakeCoralCommand();
			default:
				return null;
		}
	}

	/* ----------------------------------------------------------- */

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

		if (HardwareMap.isDriveHardwarePresent()) {
			sysRoutine = driveSystem.configureAutoSettings().newRoutine("AutoRoutine");
			generateSysRoutineMap(Filesystem.getDeployDirectory().toString());
		}

		autoPaths = new AutoPaths();
		SmartDashboard.putString("Auto State", "AUTO INIT");
	}

	/**
	 * Creates and returns a auto routine that start with a path.
	 * <br> <br>
	 * For clarification with the throwException parameter, if you set it to FALSE,
	 * it will just not add the command request into the returned SequentialCommandGroup.
	 * This could be helpful for testing series of commands when you are working with partial
	 * systems.
	 * <br> <br>
	 * If all systems are available, it should be set to TRUE - in an ideal
	 * environment, no commands should throw this issue if all systems are available. It will be
	 * more useful for debugging in this environment if you set throwException to true.
	 *
	 * @param autoStageSupply string of commands and trajectory names
	 * @param throwException whether to throw an exception at a missing/unknown command or not.
	 * @return the auto routine
	 */
	public Command generateSequentialAutoWorkflow(Object[] autoStageSupply,
		boolean throwException) {
		SequentialCommandGroup seqInstruction = new SequentialCommandGroup();
		int trajIdx = 0;

		for (int i = 0; i < autoStageSupply.length; i++) {
			var autoStage = autoStageSupply[i];

			if (autoStage.getClass().equals(String.class)) {
				/* -- Processing drive trajs -- */
				if (HardwareMap.isDriveHardwarePresent() && paths.containsKey(autoStage)) {
					AutoTrajectory traj = paths.get(autoStage);
					if (trajIdx++ == 0) {
						seqInstruction.addCommands(traj.resetOdometry());
						if (Robot.isSimulation()) {
							if (traj.getInitialPose().isPresent()) {
								initPose = traj.getInitialPose().get();
							} else {
								initPose = getStartingAlliancePosition();
							}
						}
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
							+ autoStage + " is unavailable." + "Not adding to sequential flow. ");
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
					System.out.println("Added" + autoStage.toString());
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
				String[] loggingString = new String[((Object[]) autoStage).length];
				int t = 0;

				for (Object autoParallelStage: (Object[]) autoStage) {

					/* -- Processing drive trajs -- */
					if (autoParallelStage.getClass().equals(String.class)
						&& driveSystem != null) {
						if (paths.containsKey(autoParallelStage)) {
							AutoTrajectory traj = paths.get(autoParallelStage);
							if (trajIdx++ == 0) {
								parallelQueue.addCommands(
									traj.resetOdometry().andThen(traj.cmd())
								);

								if (Robot.isSimulation()) {
									initPose = traj.getInitialPose().get();
								}

							} else {
								parallelQueue.addCommands(traj.cmd());
							}

							loggingString[t++] = (String) autoParallelStage;
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
							loggingString[t++] = autoParallelStage.toString();
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

				parallelQueue.addCommands(getAutoLogCommand(loggingString));
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
			sysRoutine.active().onTrue(
				seqInstruction
				.andThen(driveSystem.brakeCommand())
				.andThen(getAutoLogCommand(new Object[] {"AUTO COMPLETE"}))
			);

			return sysRoutine.cmd();
		} else {
			return seqInstruction;
		}
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
				SmartDashboard.putString("Auto State", Arrays.toString(cAutoState));
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

	/**
	 * Get the initial pose of the robot for simulation purposes.
	 * @return Pose2d initial pose
	 */
	public Pose2d getInitialAutoPose() {
		return initPose; //getStartingAlliancePosition();
	}

	private Pose2d getStartingAlliancePosition() {
		if (!DriverStation.getAlliance().isEmpty()) {
			if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
				switch (DriverStation.getLocation().getAsInt()) {
					case 1:
						return SimConstants.BLUE_1_STARTING_POS_M;
					case 2:
						return SimConstants.BLUE_2_STARTING_POS_M;
					case (2 + 1):
						return SimConstants.BLUE_3_STARTING_POS_M;
					default:
						return new Pose2d();
				}
			} else {
				switch (DriverStation.getLocation().getAsInt()) {
					case 1:
						return SimConstants.RED_1_STARTING_POS_M;
					case 2:
						return SimConstants.RED_2_STARTING_POS_M;
					case (2 + 1):
						return SimConstants.RED_3_STARTING_POS_M;
					default:
						return new Pose2d();
				}
			}
		} else {
			return new Pose2d();
		}
	}
}
