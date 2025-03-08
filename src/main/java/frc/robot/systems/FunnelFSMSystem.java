package frc.robot.systems;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;
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
		CLOSED
	}

	/* ======================== Private variables ======================== */
	private FunnelFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	private Servo funnelServo;
	private TimeOfFlight reefDistanceSensor;
	private DigitalInput coralBreakBeam;
	private boolean timerRunning;
	private Timer funnelClosedTimer = new Timer();

	/* ======================== Constructor ======================== */
	/**
	 * Create a FunnelFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FunnelFSMSystem() {
		// Perform hardware init
		funnelServo = new Servo(HardwareMap.FUNNEL_SERVO_PWM_PORT);
		funnelServo.set(Constants.FUNNEL_CLOSED_POS_ROTS);

		reefDistanceSensor = new TimeOfFlight(HardwareMap.FUNNEL_TOF_ID);

		coralBreakBeam = new DigitalInput(HardwareMap.FUNNEL_BREAK_BEAM_DIO_PORT);
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
		currentState = FunnelFSMState.CLOSED;

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
			case OUTTAKE:
				handleOuttakeState(input);
				break;

			case CLOSED:
				handleClosedState(input);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/**
	 * Returns the funnel timer.
	 * @return time the funnel has been opened.
	 */
	public double getTime() {
		return funnelClosedTimer.get();
	}

	/**
	 * Calls all logging and telemetry to be updated periodically.
	 */
	public void updateLogging() {
		// Telemetry and logging
		Logger.recordOutput("Funnel Position", funnelServo.get());
		Logger.recordOutput("Funnel State", currentState.toString());
		Logger.recordOutput("Distance to Reef", reefDistanceSensor.getRange());
		Logger.recordOutput("Holding Coral?", isHoldingCoral());
	}

	/**
	 * Getter for the status of the funnel's break beam.
	 * Public for access to elevator.
	 * @return whether the limit is reached
	 */
	public boolean isHoldingCoral() {
		if (Robot.isSimulation()) {
			return true;
		}
		return !coralBreakBeam.get(); // true = beam intact
		// return true; // temp always hold coral
	}

	/**
	 * Getter for the status of the funnel's distance sensor.
	 * Public for access to drive/cv.
	 * @return distance to reef from distance sensor.
	 */
	public double getDistanceToReef() {
		if (Robot.isSimulation()) {
			return 0;
		}
		return reefDistanceSensor.getRange();
	}

	/**
	 * Set the state of the FSM.
	 * @param state The state to set the FSM to.
	 */
	public void setState(FunnelFSMState state) {
		currentState = state;
	}

	/* ======================== Private methods ======================== */

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in OUTTAKE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOuttakeState(TeleopInput input) {
		if (!timerRunning) {
			timerRunning = true;
			funnelClosedTimer.reset();
			funnelClosedTimer.start();
		}

		if (funnelClosedTimer.get() >= 1) {
			funnelServo.set(Constants.FUNNEL_OUTTAKE_POS_ROTS);
		}
	}

	/**
	 * Handle behavior in CLOSED.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleClosedState(TeleopInput input) {
		funnelServo.set(Constants.FUNNEL_CLOSED_POS_ROTS);
		timerRunning = false;
	}

	/* ---- Funnel Commands ---- */

	/** A command that opens the funnel servo. */
	class IntakeCoralCommand extends Command {

		private Timer timer;

		IntakeCoralCommand() {
			timer = new Timer();
		}

		@Override
		public void initialize() {
			timer.reset();
			timer.start();
		}

		@Override
		public void execute() {
			funnelServo.set(Constants.FUNNEL_CLOSED_POS_ROTS);
		}

		@Override
		public boolean isFinished() {
			return isHoldingCoral();
			// return timer.get() > Constants.FUNNEL_INOUT_TIME_SECS || isHoldingCoral();
		}

		@Override
		public void end(boolean interrupted) {
			timer.stop();
			timer.reset();
		}
	}

	/** A command that closes the funnel servo. */
	class OuttakeCoralCommand extends Command {
		private Timer timer;

		OuttakeCoralCommand() {
			timer = new Timer();
		}

		public void initialize() {
			timer.reset();
		}

		@Override
		public void execute() {
			funnelServo.set(Constants.FUNNEL_OUTTAKE_POS_ROTS);
		}

		@Override
		public boolean isFinished() {
			//return true;
			return timer.get() >= Constants.FUNNEL_INOUT_REAL_TIME_SECS;
			// return !isHoldingCoral();
		}

		@Override
		public void end(boolean interrupted) {
			funnelServo.set(Constants.FUNNEL_CLOSED_POS_ROTS);
			timer.stop();
			timer.reset();
		}
	}

	/**
	 * Creates a Command to open the funnel.
	 * @return A new funnel open command.
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
