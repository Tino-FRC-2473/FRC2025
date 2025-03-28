package frc.robot.systems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.Constants;
import frc.robot.HardwareMap;
import frc.robot.Robot;

// WPILib Imports

// Third party Hardware Imports

// Robot Imports
import frc.robot.TeleopInput;

public class FunnelFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FunnelFSMState {
		OUTTAKE,
		IDLE
	}

	/* ======================== Private variables ======================== */
	private FunnelFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	private Servo outtakeServo;

	private DigitalInput coralBreakBeam;
	private Timer outtakeTimer = new Timer();
	private LoggedNetworkBoolean simbreakbeam;

	/* ======================== Constructor ======================== */
	/**
	 * Create a FunnelFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FunnelFSMSystem() {
		// Perform hardware init
		outtakeServo = new Servo(HardwareMap.OUTTAKE_SERVO_PWM_PORT);
		outtakeServo.set(Constants.OUTTAKE_CLOSED_POS_ROTS);

		coralBreakBeam = new DigitalInput(HardwareMap.OUTTAKE_BREAK_BEAM_DIO_PORT);

		if (Robot.isSimulation()) {
			simbreakbeam = new LoggedNetworkBoolean("Holding Coral Selector", false);
		} else {
			simbreakbeam = null;
		}

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FunnelFSMState getCurrentState() {
		return currentState;
	}
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = FunnelFSMState.IDLE;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		// Handle states
		if (input == null) {
			return;
		}
		switch (currentState) {
			case OUTTAKE -> handleOuttakeState(input);
			case IDLE -> handleIdleState(input);
			default -> throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		currentState = nextState(input);
	}

	/**
	 * Returns the funnel outtake timer value.
	 * @return amount of time the funnel has been opened.
	 */
	public double getOuttakeTimeElapsed() {
		return outtakeTimer.get();
	}

	/**
	 * Calls all logging and telemetry to be updated periodically.
	 */
	public void updateLogging() {
		// Telemetry and logging
		Logger.recordOutput("Funnel Position", outtakeServo.get());
		Logger.recordOutput("Funnel State", currentState.toString());
		Logger.recordOutput("Holding Coral?", isHoldingCoral());
	}

	/**
	 * Getter for the status of the funnel's break beam.
	 * Public for access to elevator.
	 * @return whether the limit is reached
	 */
	public boolean isHoldingCoral() {
		if (Robot.isSimulation()) {
			return simbreakbeam.get();
		}
		return !coralBreakBeam.get(); // true = beam intact
	}

	/* ======================== Private methods ======================== */
	/**
 	 * Decide the next state to transition to. This is a function of the inputs
 	 * and the current state of this FSM. This method should not have any side
 	 * effects on outputs. In other words, this method should only read or get
 	 * values to decide what state to go to.
 	 * @param input Global TeleopInput if robot in teleop mode or null if
 	 *        the robot is in autonomous mode.
 	 * @return FSM state for the next iteration
 	 */
	private FunnelFSMState nextState(TeleopInput input) {
		switch (currentState) {
			case IDLE:
				if (input.isOuttakeButtonPressed()) {
					return FunnelFSMState.OUTTAKE;
				}
				return FunnelFSMState.IDLE;

			case OUTTAKE:
				if (!input.isOuttakeButtonPressed()) {
					return FunnelFSMState.IDLE;
				}
				return FunnelFSMState.OUTTAKE;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in OUTTAKE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOuttakeState(TeleopInput input) {
		outtakeServo.set(Constants.OUTTAKE_OPEN_POS_ROTS);

		outtakeTimer.reset();
		outtakeTimer.start();
	}

	/**
	 * Handle behavior in IDLE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		if (outtakeTimer.get() >= Constants.CORAL_SCORE_TIME_SECS) {
			outtakeServo.set(Constants.OUTTAKE_CLOSED_POS_ROTS);
		}
	}

	/* ---- Funnel Commands ---- */

	/** A command that opens the funnel servo. */
	class IntakeCoralCommand extends Command {
		@Override
		public void execute() {
			outtakeServo.set(Constants.OUTTAKE_CLOSED_POS_ROTS);
		}

		@Override
		public boolean isFinished() {
			return isHoldingCoral();
		}
	}

	/** A command that closes the funnel servo. */
	class OuttakeCoralCommand extends Command {
		private Timer autoOuttakeTimer;

		OuttakeCoralCommand() {
			autoOuttakeTimer = new Timer();
		}

		public void initialize() {
			autoOuttakeTimer.reset();
		}

		@Override
		public void execute() {
			outtakeServo.set(Constants.OUTTAKE_OPEN_POS_ROTS);
		}

		@Override
		public boolean isFinished() {
			return autoOuttakeTimer.get() >= Constants.FUNNEL_INOUT_REAL_TIME_SECS;
		}

		@Override
		public void end(boolean interrupted) {
			outtakeServo.set(Constants.OUTTAKE_CLOSED_POS_ROTS);
			autoOuttakeTimer.stop();
			autoOuttakeTimer.reset();
		}
	}

	/**
	 * Creates a Command to intake coral.
	 * @return A new intake coral command.
	 */
	public Command intakeCoralCommand() {
		return new IntakeCoralCommand();
	}

	/**
	 * Creates a Command to close the funnel.
	 * @return A new funnel close command.
	 */
	public Command outtakeCoralCommand() {
		return new OuttakeCoralCommand();
	}
}
