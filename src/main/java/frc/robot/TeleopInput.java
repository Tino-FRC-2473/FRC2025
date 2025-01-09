package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.PS4Controller;

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Constants ======================== */
	private static final int MECH_CONTROLLER_PORT = 0;
	private static final int DRIVE_CONTROLLER_PORT = 1;

	/* ======================== Private variables ======================== */
	// Input objects
	private PS4Controller mechController;
	private PS4Controller driveController;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		mechController = new PS4Controller(MECH_CONTROLLER_PORT);

		driveController = new PS4Controller(DRIVE_CONTROLLER_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/**
	 * Get the value of the source elevator target button (cross).
	 * @return If the button is pressed
	 */
	public boolean isStationButtonPressed() {
		return mechController.getCrossButton();
	}
	/**
	 * Get the value of the L4 elevator target button (triangle).
	 * @return If the button is pressed
	 */
	public boolean isL4ButtonPressed() {
		return mechController.getTriangleButton();
	}
	/**
	 * Get the value of the ground elevator target button (circle).
	 * @return If the button is pressed
	 */
	public boolean isGroundButtonPressed() {
		return mechController.getCircleButton();
	}

	/**
	 * Get the manual elevator movement input (right stick Y).
	 * @return A double in the range [-1,1] representing the control input
	 */
	public double getManualElevatorMovementInput() {
		return mechController.getRightY();
	}

	/* ======================== Private methods ======================== */

}
