package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AddressableLEDStrip {
	private AddressableLED led;
	private AddressableLEDBuffer ledBuffer;
	private boolean isOn;

	private static final int LED_STRIP_PORT = 8; // TBD
	private static final int LED_STRIP_BUFFER = 3; // TBD

	/**
	 * Creates a new AddressableLEDStrip object.
	 */
	public AddressableLEDStrip() {
		led = new AddressableLED(LED_STRIP_PORT);
		ledBuffer = new AddressableLEDBuffer(LED_STRIP_BUFFER);
		led.setLength(ledBuffer.getLength());
		led.setData(ledBuffer);
		led.start();
	}

	/**
	 * Turns on all LED lights in the strip.
	 * @param r Amount of red light, from 0 to 255.
	 * @param g Amount of green light, from 0 to 255.
	 * @param b Amount of blue light, from 0 to 255.
	 */
	public void turnOn(int r, int g, int b) {
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, r, g, b);
		}
		led.setData(ledBuffer);
		isOn = true;
	}

	/**
	 * Turns off all LED lights in the strip.
	 */
	public void turnOff() {
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, 0, 0, 0);
		}
		led.setData(ledBuffer);
		isOn = false;
	}

	/**
	 * If on, turns off all LED lights in the strip. Else, turns on all LED lights.
	 * @param r Amount of red light, from 0 to 255.
	 * @param g Amount of green light, from 0 to 255.
	 * @param b Amount of blue light, from 0 to 255.
	 */
	public void toggle(int r, int g, int b) {
		if (isOn) {
			turnOff();

		} else {
			turnOn(r, g, b);
		}
	}

	/**
	 * Getter for the isOn field.
	 * @return If the LED strip is on.
	 */
	public boolean isOn() {
		return isOn;
	}

	/**
	 * Prints the state of the LED to SmartDashboard.
	 */
	public void reportToDashboard() {
		SmartDashboard.putBoolean("Led On: ", isOn);
	}
}
