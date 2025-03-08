package frc.robot.motors;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.SimulatedBattery;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.jni.CANBusJNI;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class TalonFXWrapper extends TalonFX implements LoggedMotor {

	private static final double INERTIA_CONSTANT = 0.001;

	// Components
	private final DCMotor kraken;
	// Sim model for calculations
	// getSimState() acccesses the default sim
	private final DCMotorSim motorSimModel;

	//TODO: move this to Constants.java
	public static final double LOOP_PERIOD_MS = 0.020;

	public static final double SUPPLY_VOLTAGE = 12.;

	/**
	 * Constructor with device ID.
	 * @param deviceId the CAN ID of the motor
	 */
	public TalonFXWrapper(int deviceId) {
		this(deviceId, DCMotor.getKrakenX60(1));
	}

	/**
	 * Constructor with device ID and motor type.
	 * @param deviceId the CAN ID of the motor
	 * @param motorType the motor type
	 */
	public TalonFXWrapper(int deviceId, DCMotor motorType) {
		this(deviceId, "rio", motorType);
	}

	/**
	 * Constructor with device ID and CAN bus string.
	 * @param deviceId the CAN ID of the motor
	 * @param canbus the string form of the canbus
	 */
	public TalonFXWrapper(int deviceId, String canbus) {
		this(deviceId, canbus, DCMotor.getKrakenX60(1));
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
				1.0
			),
			kraken
		);
	}

	@Override
	public void update() {
		if (Robot.isSimulation()) {
			var talonFXSim = getSimState();

			// set the supply voltage of the TalonFX
			talonFXSim.setSupplyVoltage(SUPPLY_VOLTAGE);

			// get the motor voltage of the TalonFX
			var motorVoltage = talonFXSim.getMotorVoltageMeasure();

			// use the motor voltage to calculate new position and velocity
			// using WPILib's DCMotorSim class for physics simulation
			motorSimModel.setInputVoltage(motorVoltage.in(Volts));
			motorSimModel.update(LOOP_PERIOD_MS); // assume 20 ms loop time

			// apply the new rotor position and velocity to the TalonFX;
			// note that this is rotor position/velocity (before gear ratio), but
			// DCMotorSim returns mechanism position/velocity (after gear ratio)
			talonFXSim.setRawRotorPosition(motorSimModel.getAngularPosition());
			talonFXSim.setRotorVelocity(motorSimModel.getAngularVelocity());
		}
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

	@Override
	public double getLoggedVoltage() {
		return getMotorVoltage().getValue().in(Volts);
	}
}
