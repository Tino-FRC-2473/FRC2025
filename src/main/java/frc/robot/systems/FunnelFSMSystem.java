package frc.robot.systems;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.SimConstants;
import frc.robot.logging.MechLogging;
import frc.robot.logging.SimLogging;
import static edu.wpi.first.units.Units.Degrees; import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

// import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.HardwareMap;
import frc.robot.Robot;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

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
	// private TimeOfFlight reefDistanceSensor;
	private DigitalInput coralBreakBeam;

	private boolean holdingCoral = true;

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

		coralBreakBeam = new DigitalInput(HardwareMap.FUNNEL_BREAK_BEAM_DIO_PORT);

		// reefDistanceSensor = new TimeOfFlight(HardwareMap.FUNNEL_TOF_ID);

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
	 *              the robot is in autonomous mode.
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

		// Switch state
		currentState = nextState(input);

		// Telemetry and logging
		Logger.recordOutput("Funnel Position", funnelServo.get());
		Logger.recordOutput("Funnel State", currentState.toString());

		// Logger.recordOutput("Distance to Reef", reefDistanceSensor.getRange());
		// Logger.recordOutput("Reef in Range?",
		// reefDistanceSensor.getRange() <= Constants.REEF_DISTANCE_THRESHOLD_MM);

		Logger.recordOutput("Holding Coral?", isHoldingCoral());
	}

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FunnelFSMState nextState(TeleopInput input) {
		switch (currentState) {
			case OUTTAKE:
				if (!input.isFunnelButtonPressed()) {
					return FunnelFSMState.CLOSED;
				} else {
					return FunnelFSMState.OUTTAKE;
				}

			case CLOSED:
				if (input.isFunnelButtonPressed()) {
					return FunnelFSMState.OUTTAKE;
				} else {
					return FunnelFSMState.CLOSED;
				}

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in OUTTAKE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	private void handleOuttakeState(TeleopInput input) {
		funnelServo.set(Constants.FUNNEL_OUTTAKE_POS_ROTS);

		if (Robot.isSimulation() && isHoldingCoral()) {
			dropSimCoral();
		}
	}

	/**
	 * Handle behavior in CLOSED.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	private void handleClosedState(TeleopInput input) {
		funnelServo.set(Constants.FUNNEL_CLOSED_POS_ROTS);
	}

	/**
	 * Getter for the result of the funnel's break beam.
	 * Public for access to elevator.
	 * @return whether the limit is reached
	 */
	public boolean isHoldingCoral() {
		if (Robot.isSimulation()) {
			return holdingCoral;
		}
		return !coralBreakBeam.get(); // true = beam intact
		// return true; // temp always hold coral
	}

	/**
	 * Give the funnel a coral.
	 */
	public void giveCoral() {
		holdingCoral = true;
	}

	/* ---- Funnel Commands ---- */

	/** A command that opens the funnel servo. */
	class OpenFunnelCommand extends Command {
		OpenFunnelCommand() {
		}

		@Override
		public void execute() {
			funnelServo.set(Constants.FUNNEL_OUTTAKE_POS_ROTS);

		}

		@Override
		public boolean isFinished() {
			return !isHoldingCoral(); // done when no coral
		}

		@Override
		public void end(boolean interrupted) {
		}
	}

	/** A command that closes the funnel servo. */
	class CloseFunnelCommand extends Command {
		private Timer timer;

		CloseFunnelCommand() {
			timer = new Timer();
			timer.start();
		}

		@Override
		public void execute() {
			funnelServo.set(Constants.FUNNEL_CLOSED_POS_ROTS);
		}

		@Override
		public boolean isFinished() {
			return timer.get() >= Constants.FUNNEL_CLOSE_TIME_SECS;
		}

		@Override
		public void end(boolean interrupted) {
			timer.stop();
		}
	}

	/**
	 * Creates a Command to open the funnel.
	 * @return A new funnel open command.
	 */
	public Command openFunnelCommand() {
		return new OpenFunnelCommand();
	}

	/**
	 * Creates a Command to close the funnel.
	 * @return A new funnel close command.
	 */
	public Command closeFunnelCommand() {
		return new CloseFunnelCommand();
	}

	/**
	 * Drops the coral from the funnel.
	 */
	public void dropSimCoral() {
		if (!isHoldingCoral()) {
			return;
		}

		var simPose = SimLogging.getInstance().getSimRobotPose();

		SimulatedArena.getInstance()
				.addGamePieceProjectile(new ReefscapeCoralOnFly(
						// Obtain robot position from drive simulation
						simPose.getTranslation(),
						// The scoring mechanism is installed at (0.46, 0) (meters) on the robot
						new Translation2d(
							SimConstants.WIDTH_IN / 2 * simPose.getRotation().getSin(), 0),
						// Obtain robot speed from drive simulation
						SimLogging.getInstance().getSimRobotChassisSpeeds(),
						// Obtain robot facing from drive simulation
						simPose.getRotation(),
						// The height at which the coral is ejected
						Meters.of(MechLogging.getInstance().getElevatorStage3().getZ()),
						// The initial speed of the coral
						MetersPerSecond.of(SimConstants.FUNNEL_OUTTAKE_INIT_SPD_MPS),
						// The coral is ejected vertically downwards
						Degrees.of(SimConstants.FUNNEL_OUTTAKE_ROT_DEG)));
		holdingCoral = false;
	}
}
