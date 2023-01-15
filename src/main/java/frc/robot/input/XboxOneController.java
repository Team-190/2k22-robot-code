package frc.robot.input;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class XboxOneController extends Joystick {
    /**
    * Initializes an Xbox One controller on a specific channel, mapping the buttons
    *
    * @param port the port the controller is plugged into
    */
    public XboxOneController(int port) {
        super(port);
    }

    // Map each button on the Controller to a Joystick Button Object
    public JoystickButton xButton = new JoystickButton(this, 3);
    public JoystickButton yButton = new JoystickButton(this, 4);
    public JoystickButton aButton = new JoystickButton(this, 1);
    public JoystickButton bButton = new JoystickButton(this, 2);
    public JoystickButton rightBumper = new JoystickButton(this, 6);
    public JoystickButton leftBumper = new JoystickButton(this, 5);
    public JoystickButton startButton = new JoystickButton(this, 8);
    public JoystickButton selectButton = new JoystickButton(this, 7);
    public JoystickButton leftStickButton = new JoystickButton(this, 9);
    public JoystickButton rightStickButton = new JoystickButton(this, 10);

    private int m_outputs;
    private short m_leftRumble;
    private short m_rightRumble;

    /**
    * Gets the state of the Left Bumper for use in Quick Turn
    *
    * @return boolean True or False
    */
    public boolean getLeftBumper() {
        return leftBumper.getAsBoolean();
    }

    /**
    * Get the position of the Left Stick X Axis
    *
    * @return the value of the axis
    */
    public double getLeftStickX() {
        return this.getRawAxis(0);
    }

    /**
    * Gets the position of the Left Stick Y Axis
    *
    * @return the value of the axis
    */
    public double getLeftStickY() {
        // Forward axis is negative so negate in code
        return -this.getRawAxis(1);
    }

    /**
    * Gets the position of the Left Trigger
    *
    * @return the value of the trigger axis
    */
    public double getLeftTrigger() {
        return this.getRawAxis(2);
    }

    /**
    * Gets the position of the Right Trigger
    *
    * @return the value of the trigger axis
    */
    public double getRightTrigger() {
        return this.getRawAxis(3);
    }

    /**
    * Gets the position of the Right X Axis
    *
    * @return the value of the axis
    */
    public double getRightStickX() {
        return this.getRawAxis(4);
    }

    /**
    * Gets the position of the Right Y Axis
    *
    * @return the value of the axis
    */
    public double getRightStickY() {
        // Forward axis is negative so negate in code
        return -this.getRawAxis(5);
    }

    /**
    * Gets the position of the DPad X Axis
    *
    * @return the value of the axis
    */
    public double getDPayX() {
        return this.getRawAxis(6);
    }

    /**
    * Gets the position of the DPad Y Axis
    *
    * @return the value of the axis
    */
    public double getDPadY() {
        return this.getRawAxis(7);
    }

    /**
    * Sets the magnitude of rumble in the controller
    *
    * @param leftValue the left rumble value
    * @param rightValue the right rumble value
    */
    public void setRumble(double leftValue, double rightValue) {
        setRumble(RumbleType.kLeftRumble, leftValue);
        setRumble(RumbleType.kRightRumble, rightValue);
    }
}
