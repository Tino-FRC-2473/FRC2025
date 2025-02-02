package frc.robot.motors;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;


public class MotorManager {

	/** A global list containing all logged motors. */
	private static List<LoggedMotor> motorList = new LinkedList<>();

	/**
	 * Updates all motors in the global list and logs information to Ascope.
	 */
	public static void update() {
		for (LoggedMotor motor : motorList) {
			// Update motor
			motor.update();

			// Log motor information
			Logger.recordOutput("Motor " + motor.getIdentifier()
					+ " Rotations", motor.getLoggedPosition());
			Logger.recordOutput("Motor " + motor.getIdentifier()
					+ " Velocity", motor.getLoggedVelocity());
			Logger.recordOutput("Motor " + motor.getIdentifier()
					+ " Setpoint", motor.getLoggedSetpoint());
		}
	}

	/**
	 * Resets the motor list to an empty list.
	 */
	public static void reset() {
		motorList.clear();
	}

	/**
	 * Adds a motor to the motor list.
	 * @param motor the motor to add
	 */
	public static void addMotor(LoggedMotor motor) {
		motorList.add(motor);
	}
}
