package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED {
	private AddressableLED led;
	private AddressableLEDBuffer ledBuffer;
	private int rainbowFirstPixelHue = 0;
	private int greenVal = 0;
	private boolean forward = true;
	private static final int LED_PORT = 9;
	private static final int LED_BUFFER_LENGTH = 30;
	private static final int ONE_HUNDRED_EIGHTY = 180;
	private static final int ONE_HUNDRED = 100;
	private static final int RAINBOW_PIXEL_INCREASE = 3;

	private static final int GREEN_RGB_R = 135;
	private static final int GREEN_RGB_G = 255;
	private static final int GREEN_RGB_B = 8;

	private static final int ORANGE_RGB_R = 255;
	private static final int ORANGE_RGB_G = 92;
	private static final int ORANGE_RGB_B = 5;

	private static final int RED_RGB_R = 139;
	private static final int RED_RGB_G = 0;
	private static final int RED_RGB_B = 0;

	private static final int CR_RGB_R = 255;

	private static final int RAINBOW_S = 255;
	private static final int RAIBOW_V = 128;

	// Objective: orange ==> intake coral, green ==>
	// aligned to april tag, red ==> not aligned to april tag.

	/**
	* Constructs LED object.
	*/
	public LED() {
		led = new AddressableLED(LED_PORT);
		ledBuffer = new AddressableLEDBuffer(LED_BUFFER_LENGTH);
		led.setLength(ledBuffer.getLength());
		led.start();
	}

	/**
	* Changes the LED color to green.
	*/
	public void greenLight() {
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, GREEN_RGB_R, GREEN_RGB_G, GREEN_RGB_B);
		}
		led.setData(ledBuffer);
	}
	/**
	* Changes the LED color to red.
	*/
	public void redLight() {
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, RED_RGB_R, RED_RGB_G, RED_RGB_B);
		}
		led.setData(ledBuffer);
	}

	/**
	* Changes the LED color to orange.
	*/
	public void orangeLight() {
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, ORANGE_RGB_R, ORANGE_RGB_G, ORANGE_RGB_B);
		}
		led.setData(ledBuffer);
	}

	/**
	* Turns off the LEDs.
	*/
	public void turnOff() {
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, 0, 0, 0);
		}
		led.setData(ledBuffer);
	}

	/**
	 * Updates the LED buffer by setting the red channel to the value of CR_RGB_R,
	 * while dynamically adjusting the green channel between 0 and ONE_HUNDRED.
	 * The transition is determined by a directional flag `forward` which toggles
	 * when the green value reaches its bounds.
	 *
	 * TL;DR: Green channel value is incremented or decremented based on the directional flag.
	 * <pre>
	 *   - Increases the green channel value if `forward` is true. Otherwise, decreases it.
	 * 	 - Updates the LED data with the modified buffer after each operation.
	 * </pre>
	 */
	public void cr() {
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, CR_RGB_R, greenVal, 0);
		}

		if (greenVal >= ONE_HUNDRED) {
			forward = false;
		} else if (greenVal <= 0) {
			forward = true;
		}

		if (forward) {
			greenVal++;
		} else {
			greenVal--;
		}

		led.setData(ledBuffer);
	
	}

	/**
	* Sets the LED color to rainbow.
	*/
	public void rainbow() {
		// For every pixel
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			// Calculate the hue - hue is easier for rainbows because the color
			// shape is a circle so only one value needs to precess
			final var hue = (rainbowFirstPixelHue + (i * ONE_HUNDRED_EIGHTY
				/ ledBuffer.getLength())) % ONE_HUNDRED_EIGHTY;
			// Set the value
			ledBuffer.setHSV(i, hue, RAINBOW_S, RAIBOW_V);
		}
		// Increase by to make the rainbow "move"
		rainbowFirstPixelHue += RAINBOW_PIXEL_INCREASE;

		// Check bounds
		rainbowFirstPixelHue %= ONE_HUNDRED_EIGHTY;
		led.setData(ledBuffer);
	}

	/**
	* Sets RGB value of LED.
	* @param r
	* @param g
	* @param b
	*/
	public void setRGBVals(int r, int g, int b) {
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			// Sets the specified LED to the RGB values for red
			ledBuffer.setRGB(i, r, g, b);
		}
		led.setData(ledBuffer);
	}

	/**
	* Sets HSV value of LED.
	* @param hue
	* @param saturation
	* @param val
	*/
	public void setHSVVals(int hue, int saturation, int val) {
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			// Sets the specified LED to the HSV values for red
			ledBuffer.setHSV(i, hue, saturation, val);
		}

		led.setData(ledBuffer);
	}

}
