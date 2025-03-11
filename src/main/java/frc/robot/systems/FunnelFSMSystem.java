package frc.robot.systems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
		INTAKE,
		OUTTAKE,
		IDLE
	}

	/* ======================== Private variables ======================== */
	private FunnelFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	private Servo outtakeServo;
	private TalonFX intakeMotor;

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
		outtakeServo = new Servo(HardwareMap.OUTTAKE_SERVO_PWM_PORT);
		outtakeServo.set(Constants.OUTTAKE_CLOSED_POS_ROTS);

		// a lotta code to config a motor, thanks ctre
		intakeMotor = new TalonFX(HardwareMap.CAN_ID_INTAKE);
		var talonFXConfigs = new TalonFXConfiguration();
		var outputConfigs = talonFXConfigs.MotorOutput;

		outputConfigs.NeutralMode = NeutralModeValue.Coast;
		intakeMotor.getConfigurator().apply(talonFXConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
				Constants.UPDATE_FREQUENCY_HZ,
				intakeMotor.getVelocity(),
				intakeMotor.getAcceleration(),
				intakeMotor.getMotorVoltage()
		);

		intakeMotor.optimizeBusUtilization();

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
			case INTAKE:
				handleIntakeState(input);
				break;

			case OUTTAKE:
				handleOuttakeState(input);
				break;

			case IDLE:
				handleIdleState(input);
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
			return true;
		}
		return !coralBreakBeam.get(); // true = beam intact
	}

	/**
	 * Set the state of the FSM.
	 * @param state The state to set the FSM to.
	 */
	public void setState(FunnelFSMState state) {
		currentState = state;
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
				} else if (isHoldingCoral()) {
					return FunnelFSMState.INTAKE;
				}
				return FunnelFSMState.IDLE;

			case OUTTAKE:
				if (!input.isOuttakeButtonPressed()) {
					return FunnelFSMState.IDLE;
				}
				return FunnelFSMState.OUTTAKE;

			case INTAKE:
				if (isHoldingCoral()) {
					return FunnelFSMState.IDLE;
				}
				return FunnelFSMState.INTAKE;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in INTAKE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIntakeState(TeleopInput input) {
		intakeMotor.setVoltage(Constants.INTAKE_VOLTAGE);
		outtakeServo.set(Constants.OUTTAKE_CLOSED_POS_ROTS);
		timerRunning = false;
	}

	/**
	 * Handle behavior in OUTTAKE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOuttakeState(TeleopInput input) {
		intakeMotor.setVoltage(0);

		if (!timerRunning) {
			timerRunning = true;
			funnelClosedTimer.reset();
			funnelClosedTimer.start();
		}

		if (funnelClosedTimer.get() >= 1) {
			outtakeServo.set(Constants.OUTTAKE_OPEN_POS_ROTS);
		}
	}

	/**
	 * Handle behavior in CLOSED.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		intakeMotor.setVoltage(0);
		outtakeServo.set(Constants.OUTTAKE_CLOSED_POS_ROTS);
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
			outtakeServo.set(Constants.OUTTAKE_CLOSED_POS_ROTS);
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
			outtakeServo.set(Constants.OUTTAKE_OPEN_POS_ROTS);
		}

		@Override
		public boolean isFinished() {
			//return true;
			return timer.get() >= Constants.FUNNEL_INOUT_REAL_TIME_SECS;
			// return !isHoldingCoral();
		}

		@Override
		public void end(boolean interrupted) {
			outtakeServo.set(Constants.OUTTAKE_CLOSED_POS_ROTS);
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
