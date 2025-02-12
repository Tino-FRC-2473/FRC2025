package frc.robot;

// WPILib Imports

// Third party Hardware Imports
import edu.wpi.first.wpilibj.motorcontrol.Spark;
// Robot Imports
import frc.robot.constants.LEDConstants;

public class BlinkinLED {
	private Spark ledController;

	/**
	 * Constructor for BlinkinLED.
	 * Initializes the LED controller.
	 */
	public BlinkinLED() {
		// Perform hardware init
		ledController = new Spark(HardwareMap.LED_STRIP_PWM_PORT);
		ledController.set(LEDConstants.LED_BLACK_SOLID);
	}

	/**
	 * Sets the LED color to solid red.
	 */
	public void setLEDRed() {
		ledController.set(LEDConstants.LED_RED_SOLID);
	}

	/**
	 * Sets the LED color to solid Off.
	 */
	public void setLEDOff() {
		ledController.set(LEDConstants.LED_BLACK_SOLID);
	}

	/**
	 * Sets the LED color to solid green.
	 */
	public void setLEDGreen() {
		ledController.set(LEDConstants.LED_GREEN_SOLID);
	}

	/**
	 * Sets the LED color to solid Blue.
	 */
	public void setLEDBlue() {
		ledController.set(LEDConstants.LED_BLUE_SOLID);
	}

}
