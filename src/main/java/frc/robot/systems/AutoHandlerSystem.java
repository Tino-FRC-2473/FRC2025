package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HardwareMap;

public class AutoHandlerSystem {
	/* ======================== Constants ======================== */
	// Auto FSM state definitions
	public enum AutoFSMState {
		STATE1,
		STATE2,
		STATE3
	}
	public enum AutoPath {
		PATH1,
		PATH2,
		PATH3
	}

	/* ======================== Private variables ======================== */
	//Contains the sequential list of states in the current auto path that must be executed
	private AutoFSMState[] currentStateList;

	//The index in the currentStateList where the currentState is at
	private int currentStateIndex;

	//FSM Systems that the autoHandlerFSM uses
	private DriveFSMSystem driveSystem;
	private FunnelFSMSystem elevatorSystem;
	private ElevatorFSMSystem funnelSystem;

	//Predefined auto paths
	private static final AutoFSMState[] PATH1 = new AutoFSMState[]{
		AutoFSMState.STATE1, AutoFSMState.STATE2, AutoFSMState.STATE3};

	private static final AutoFSMState[] PATH2 = new AutoFSMState[]{
		AutoFSMState.STATE3, AutoFSMState.STATE2, AutoFSMState.STATE1};

	private static final AutoFSMState[] PATH3 = new AutoFSMState[]{
		AutoFSMState.STATE1, AutoFSMState.STATE3, AutoFSMState.STATE2};
	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state.
	 * Initializes any subsystems such as driveFSM, armFSM, ect.
	 * @param fsm1 the first subsystem that the auto handler will call functions on
	 * @param fsm2 the second subsystem that the auto handler will call functions on
	 * @param fsm3 the third subsystem that the auto handler will call functions on
	 */
	public AutoHandlerSystem(DriveFSMSystem fsm1, FunnelFSMSystem fsm2, ElevatorFSMSystem fsm3) {
		driveSystem = fsm1;
		elevatorSystem = fsm2;
		funnelSystem = fsm3;
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public AutoFSMState getCurrentState() {
		return currentStateList[currentStateIndex];
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 * @param path the auto path to be executed
	 */
	public void reset(AutoPath path) {
		if (HardwareMap.isDriveHardwarePresent()) {
			driveSystem.reset();
		}

		if (HardwareMap.isElevatorHardwarePresent()) {
			elevatorSystem.reset();
		}

		if (HardwareMap.isFunnelHardwarePresent()) {
			funnelSystem.reset();
		}

		currentStateIndex = 0;
		if (path == AutoPath.PATH1) {
			currentStateList = PATH1;
		} else if (path == AutoPath.PATH2) {
			currentStateList = PATH2;
		} else if (path == AutoPath.PATH3) {
			currentStateList = PATH3;
		}
	}

	/**
	 * This function runs the auto's current state.
	 */
	public void update() {
		if (currentStateIndex >= currentStateList.length) {
			return;
		}

		boolean isCurrentStateFinished = true;
		SmartDashboard.putString("Auto State", getCurrentState().toString());

		isCurrentStateFinished &= driveSystem.updateAutonomous(getCurrentState());
		isCurrentStateFinished &= elevatorSystem.updateAutonomous(getCurrentState());
		isCurrentStateFinished &= funnelSystem.updateAutonomous(getCurrentState());

		if (isCurrentStateFinished) {
			currentStateIndex++;
		}
	}
}
