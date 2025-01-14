package frc.robot.motors;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;

public class SparkMaxWrapper extends SparkMax implements LoggedMotor {

	// Components
	private final DCMotor motorType;
	private final SparkMaxSim motorSim;

	//TODO: move this to Constants.java
	public static final double LOOP_PERIOD_MS = 0.020;

	private double targetVelocity = 0;

	/**
	 * Constructor for CAN ID and motor type.
	 * @param deviceId the CAN ID of the motor
	 * @param type the type of motor
	 */
	public SparkMaxWrapper(int deviceId, MotorType type) {
		// Initialize motor
		super(deviceId, type);
		init();

		// Create sim instance
		motorType = DCMotor.getNEO(1);
		motorSim = new SparkMaxSim(this, motorType);
	}

	/**
	 * Constructor for CAN ID, motor type and an instance of DCMotor.
	 * @param deviceId the CAN ID of the motor
	 * @param type the type of motor
	 * @param dcMotor the instance of DCMotor to use for sim calculations
	 */
	public SparkMaxWrapper(int deviceId, MotorType type, DCMotor dcMotor) {
		// Initialize motor
		super(deviceId, type);
		init();

		// Create sim instance
		motorType = dcMotor;
		motorSim = new SparkMaxSim(this, motorType);
	}

	@Override
	public void update() {
		// Update sim instance
		motorSim.iterate(targetVelocity, RobotController.getBatteryVoltage(), LOOP_PERIOD_MS);
	}

	@Override
	public void set(double speed) {
		// Set real motor speed
		super.set(speed);

		// Add speed to buffer for sim
		this.targetVelocity = speed * motorType.freeSpeedRadPerSec;
	}

	@Override
	public String getIdentifier() {
		return Integer.toString(this.getDeviceId());
	}

	@Override
	public double getLoggedPosition() {
		// Sim motor
		if (Robot.isSimulation()) {
			return motorSim.getPosition();
		}

		// Real motor
		return this.getEncoder().getPosition();
	}

	@Override
	public double getLoggedVelocity() {
		// Sim motor
		if (Robot.isSimulation()) {
			return motorSim.getVelocity();
		}

		// Real motor
		return this.getEncoder().getVelocity();
	}

	@Override
	public double getLoggedSetpoint() {
		return motorSim.getSetpoint();
	}

}
