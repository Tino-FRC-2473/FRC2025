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
	private static final int DRIVE_CONTROLLER_PORT = 0;
	private static final int MECH_CONTROLLER_PORT = 1;

	/* ======================== Private variables ======================== */
	// Input objects
	private PS4Controller driveController;
	private PS4Controller mechController;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		driveController = new PS4Controller(DRIVE_CONTROLLER_PORT);
		mechController = new PS4Controller(MECH_CONTROLLER_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Drive Controller ------------------------ */
	/**
	 * Get X axis of Drive Controller.
	 * @return Axis value
	 */
	public double getDriveLeftJoystickX() {
		return driveController.getLeftX();
	}
	/**
	 * Get Y axis of Drive Controller.
	 * @return Axis value
	 */
	public double getDriveLeftJoystickY() {
		return driveController.getLeftY();
	}
	/**
	 * Get Y axis of Drive Controller.
	 * @return Axis value
	 */
	public double getDriveRightJoystickX() {
		return driveController.getRightX();
	}
	/**
	 * Get Triangle Button Pressed for Drive Controller.
	 * @return Axis value
	 */
	public boolean getDriveTriangleButton() {
		return driveController.getTriangleButton();
	}
	/**
	 * Get Circle Button Pressed for Drive Controller.
	 * @return Axis value
	 */
	public boolean getDriveCircleButton() {
		return driveController.getCircleButton();
	}
		/**
	 * Get Share Button Pressed for Drive Controller.
	 * @return Axis value
	 */
	public boolean getDriveBackButtonPressed() {
		return driveController.getShareButton();
	}

	/* ------------------------ Right Joystick ------------------------ */
	/**
	 * Get X axis of Right Joystick.
	 * @return Axis value
	 */
	public double getMechLeftJoystickX() {
		return mechController.getLeftX();
	}
	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getMechLeftJoystickY() {
		return mechController.getLeftY();
	}

	/* ======================== Private methods ======================== */

}
