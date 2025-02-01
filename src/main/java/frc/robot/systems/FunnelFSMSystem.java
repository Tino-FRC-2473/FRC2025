package frc.robot.systems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
	private SparkMax funnelMotor;
	private SparkMaxConfig motorConfig;
	private ClosedLoopConfig closedLoopConfig;
	private SparkClosedLoopController pidConroller;

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
		funnelMotor = new SparkMax(HardwareMap.CAN_ID_FUNNEL, MotorType.kBrushless);
		motorConfig = new SparkMaxConfig();
		closedLoopConfig = motorConfig.closedLoop;
		pidConroller = funnelMotor.getClosedLoopController();

		funnelMotor.getEncoder().setPosition(0);

		closedLoopConfig
			.pid(Constants.FUNNEL_MOTOR_P, Constants.FUNNEL_MOTOR_D, 0, ClosedLoopSlot.kSlot0)
			.iZone(0)
			.maxOutput(1)
			.minOutput(-1);

		motorConfig
			.idleMode(IdleMode.kBrake)
			.smartCurrentLimit(Constants.FUNNER_MOTOR_CURRENT_LIMIT)
			.apply(closedLoopConfig);

		funnelMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters,
				PersistMode.kPersistParameters);

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

		// Switch state
		currentState = nextState(input);

		// Telemetry and logging

		SmartDashboard.putNumber("Funnel Position", funnelServo.get());
		SmartDashboard.putString("Funnel State", currentState.toString());

		SmartDashboard.putNumber("Distance to Reef", reefDistanceSensor.getRange());
		SmartDashboard.putBoolean("Reef in Range?",
			reefDistanceSensor.getRange() <= Constants.REEF_DISTANCE_THRESHOLD_MM);

		SmartDashboard.putBoolean("Holding Coral?", isHoldingCoral());
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
	 *        the robot is in autonomous mode.
	 */
	private void handleOuttakeState(TeleopInput input) {
		pidConroller.setReference(Constants.FUNNEL_OUTTAKE_POS_ROTS,
			ControlType.kPosition, ClosedLoopSlot.kSlot0);
		funnelServo.set(Constants.FUNNEL_OUTTAKE_POS_ROTS);
	}
	/**
	 * Handle behavior in CLOSED.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleClosedState(TeleopInput input) {
		pidConroller.setReference(Constants.FUNNEL_CLOSED_POS_ROTS,
			ControlType.kPosition, ClosedLoopSlot.kSlot0);
		funnelServo.set(Constants.FUNNEL_CLOSED_POS_ROTS);
	}

	/**
	 * Getter for the result of the funnel's break beam.
	 * Public for access to elevator.
	 * @return whether the limit is reached
	 */
	public boolean isHoldingCoral() {
		if (Robot.isSimulation()) {
			return false;
		}
		return !coralBreakBeam.get(); // true = beam intact
	}

	/* ---- Funnel Commands ---- */

	/** A command that opens the funnel servo. */
	class OpenFunnelCommand extends Command {
		OpenFunnelCommand() { }

		@Override
		public void execute() {
			funnelServo.set(Constants.FUNNEL_OUTTAKE_POS_ROTS);
		}

		@Override
		public boolean isFinished() {
			return !isHoldingCoral(); // done when no coral
		}

		@Override
		public void end(boolean interrupted) { }
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
}
