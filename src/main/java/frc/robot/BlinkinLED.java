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

	/* LED color setters for drivers */

	/**
	 * Sets the LED color to solid green.
	 * Used when aligned to the reef.
	 */
	public void setReefAlignColor() {
		ledController.set(LEDConstants.LED_GREEN_SOLID);
	}

	/**
	 * Sets the LED color to solid sky blue.
	 * Used when aligned to the coral station.
	 */
	public void setStationAlignedColor() {
		ledController.set(LEDConstants.LED_SKY_BLUE_SOLID);
	}

	/**
	 * Sets the LED color to solid hot pink.
	 * Used when offset from an apriltag.
	 */
	public void setOffsetColor() {
		ledController.set(LEDConstants.LED_HOT_PINK_SOLID);
	}

	/**
	 * Sets the LED color to blue heartbeat.
	 * Used when robot is holding a coral.
	 */
	public void setYesCoralColor() {
		ledController.set(LEDConstants.LED_GREEN_SOLID);
	}

	/**
	 * Sets the LED color to solid red.
	 * Used when robot is not holding a coral.
	 */
	public void setNoCoralColor() {
		ledController.set(LEDConstants.LED_RED_SOLID);
	}

	/**
	 * Sets the LED color to bpm ocean.
	 * Used when robot is in autonomous.
	 */
	public void setAutoColor() {
		ledController.set(LEDConstants.LED_BPM_OCEAN_PALETTE);
	}

	/**
	 * Sets the LED color to rainbow.
	 * Used when robot is climbing.
	 */
	public void setClimbColor() {
		ledController.set(LEDConstants.LED_RAINBOW_RAINBOW_PALETTE);
	}
}
