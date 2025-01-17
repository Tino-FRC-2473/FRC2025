package frc.robot.motors;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonFXWrapper extends TalonFX implements LoggedMotor {

	// Attributes
	private static final double K_GEAR_RATIO = 1.0;
	private static final double INERTIA_CONSTANT = 0.001;

	// Components
	private final DCMotor kraken;
	// Sim model for calculations
	// getSimState() acccesses the default sim
	private final DCMotorSim motorSimModel;

	//TODO: move this to Constants.java
	public static final double LOOP_PERIOD_MS = 0.020;

	/**
	 * Constructor with device ID.
	 * @param deviceId the CAN ID of the motor
	 */
	public TalonFXWrapper(int deviceId) {
		this(deviceId, DCMotor.getKrakenX60Foc(1));
	}

	/**
	 * Constructor with device ID and motor type.
	 * @param deviceId the CAN ID of the motor
	 * @param motorType the motor type
	 */
	public TalonFXWrapper(int deviceId, DCMotor motorType) {
		this(deviceId, "", motorType);
	}

	/**
	 * Constructor with device ID and CAN bus string.
	 * @param deviceId the CAN ID of the motor
	 * @param canbus the string form of the canbus
	 */
	public TalonFXWrapper(int deviceId, String canbus) {
		this(deviceId, canbus, DCMotor.getKrakenX60Foc(1));
	}

	/**
	 * Constructor with device ID, CAN bus string, and motor type.
	 * @param deviceId the CAN ID of the motor
	 * @param canbus the string form of the canbus
	 * @param motorType the motor type
	 */
	public TalonFXWrapper(int deviceId, String canbus, DCMotor motorType) {
		// Initialize motor
		super(deviceId, canbus);
		init();

		// Create sim instance
		kraken = motorType;
		motorSimModel = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				kraken,
				INERTIA_CONSTANT,
				K_GEAR_RATIO
			),
			kraken
		);
	}

	@Override
	public void update() {
		var talonFXSim = getSimState();

		// set the supply voltage of the TalonFX
		talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
		SmartDashboard.putNumber("battery voltage", RobotController.getBatteryVoltage());

		// get the motor voltage of the TalonFX
		var motorVoltage = talonFXSim.getMotorVoltageMeasure();

		// use the motor voltage to calculate new position and velocity
		// using WPILib's DCMotorSim class for physics simulation
		motorSimModel.setInputVoltage(motorVoltage.in(Volts));
		motorSimModel.update(LOOP_PERIOD_MS); // assume 20 ms loop time

		// apply the new rotor position and velocity to the TalonFX;
		// note that this is rotor position/velocity (before gear ratio), but
		// DCMotorSim returns mechanism position/velocity (after gear ratio)
		talonFXSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(K_GEAR_RATIO));
		talonFXSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(K_GEAR_RATIO));
	}

	@Override
	public String getIdentifier() {
		return Integer.toString(getDeviceID());
	}

	@Override
	public double getLoggedPosition() {
		return this.getPosition().getValue().in(Rotations);
	}

	@Override
	public double getLoggedVelocity() {
		return this.getVelocity().getValue().in(RPM);
	}

	@Override
	public double getLoggedSetpoint() {
		return get();
	}
}
