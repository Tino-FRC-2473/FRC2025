package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

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
	private PS4Controller mechController;
	private XboxController driveController;
	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		driveController = new XboxController(DRIVE_CONTROLLER_PORT);
		mechController = new PS4Controller(MECH_CONTROLLER_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
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
	 * Get X axis of Drive Controller right.
	 * @return Axis value
	 */
	public double getDriveRightJoystickX() {
		return driveController.getRightX();
	}
	/**
	 * Get Y axis of Drive Controller right.
	 * @return Axis value
	 */
	public double getDriveRightJoystickY() {
		return driveController.getRightY();
	}
	/**
	 * Get Triangle Button Pressed for Drive Controller.
	 * @return Axis value
	 */
	public boolean getDriveTriangleButton() {
		return driveController.getAButton();
	}
	/**
	 * Get Square Button Pressed for Drive Controller.
	 * @return Axis value
	 */
	public boolean getDriveSquareButton() {
		return driveController.getYButton();
	}
	/**
	 * Get Circle Button Pressed for Drive Controller.
	 * @return Axis value
	 */
	public boolean getDriveCircleButton() {
		return driveController.getXButton();
	}
		/**
	 * Get Share Button Pressed for Drive Controller.
	 * @return Axis value
	 */
	public boolean getDriveBackButtonPressed() {
		return driveController.getLeftBumperButton();
	}

	/* ------------------------ Mech Controller ------------------------ */

	/**
	 * Get the value of the source elevator target button (cross).
	 * @return If the button is pressed
	 */
	public boolean isL2ButtonPressed() {
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
	 * Get the value of the L3 elevator target button (square).
	 * @return If the button is pressed
	 */
	public boolean isL3ButtonPressed() {
		return mechController.getSquareButton();
	}

	/**
	 * Gets the value of the options button.
	 * Intended to signify when the climber should go to the next state.
	 * @return If the options button was pressed this tick
	 */
	public boolean isClimbAdvanceStateButtonPressed() {
		return mechController.getOptionsButtonPressed();
	}

	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getManualElevatorMovementInput() {
		return -mechController.getRightY(); //up is negative y, negate for simplicity
	}

	/**
	 * Gets the value of the L1 button.
	 * Intended to signify when the climber should go to the next state.
	 * @return If the L1 button was pressed this tick
	 */
	public boolean isFunnelButtonPressed() {
		return mechController.getL1Button();
	}

	/* ======================== Private methods ======================== */

}
