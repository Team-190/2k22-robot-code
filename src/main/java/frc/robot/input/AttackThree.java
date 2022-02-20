package frc.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class AttackThree extends Joystick {

    /*
    * Total amount of buttons on the Attack Three
    */
    private final short totalButtons = 11;

    /*
    * Array of every button on the Attack Three
    */
    private JoystickButton[] buttons;

    /*
    * Amount of deadband
    */
    private double deadband = .1;

    /**
    * Initializes a Joystick on a specific channel, mapping the buttons. The Joystick will never
    * return a value in between +/- the deadband value.
    *
    * @param channel the channel the Joystick is plugged into
    */
    public AttackThree(int channel) {
        super(channel);
    }

    public JoystickButton triggerButton = new JoystickButton(this, 1);
    public JoystickButton bottomFaceButton = new JoystickButton(this, 2);
    public JoystickButton middleFaceButton = new JoystickButton(this, 3);
    public JoystickButton leftFaceButton = new JoystickButton(this, 4);
    public JoystickButton rightFaceButton = new JoystickButton(this, 5);
    public JoystickButton topLeftButton = new JoystickButton(this, 6);
    public JoystickButton bottomLeftButton = new JoystickButton(this, 7);
    public JoystickButton middleLeftButton = new JoystickButton(this, 8);
    public JoystickButton middleRightButton = new JoystickButton(this, 9);
    public JoystickButton bottomRightButton = new JoystickButton(this, 10);
    public JoystickButton topRightButton = new JoystickButton(this, 11);

    /**
     * See if trigger button is pressed
     * @return true if pressed else false
     */
    public boolean getTriggerButton() {
        return triggerButton.get();
    }

    /**
     * See if bottom face button is pressed
     * @return true if pressed else false
     */
    public boolean getBottomFaceButton() {
        return bottomFaceButton.get();
    }

    /**
     * See if middle face button is pressed
     * @return true if pressed else false
     */
    public boolean getMiddleFaceButton() {
        return middleFaceButton.get();
    }

    /**
     * See if left face button is pressed
     * @return true if pressed else false
     */
    public boolean getLeftFaceButton() {
        return leftFaceButton.get();
    }

    /**
     * See if right face button is pressed
     * @return true if pressed else false
     */
    public boolean getRightFaceButton() {
        return rightFaceButton.get();
    }

    /**
     * See if top left button is pressed
     * @return true if pressed else false
     */
    public boolean getTopLeftButton() {
        return topLeftButton.get();
    }

    /**
     * See if bottom left button is pressed
     * @return true if pressed else false
     */
    public boolean getBottomLeftButton() {
        return bottomLeftButton.get();
    }

    /**
     * See if middle left button is pressed
     * @return true if pressed else false
     */
    public boolean getMiddleLeftButton() {
        return middleLeftButton.get();
    }

    /**
     * See if middle right button is pressed
     * @return true if pressed else false
     */
    public boolean getMiddleRightButton() {
        return middleRightButton.get();
    }

    /**
     * See if bottom right button is pressed
     * @return true if pressed else false
     */
    public boolean getBottomRightButton() {
        return bottomRightButton.get();
    }

    /**
     * See if top right button is pressed
     * @return true if pressed else false
     */
    public boolean getTopRightButton() {
        return topRightButton.get();
    }

    /**
    * Gets position of a specific axis, accounting for the deadband
    *
    * @param axis the AxisType to retrieve
    * @return the value of the axis, with the deadband factored in
    */
    public double getAxis(AttackThreeAxis axis) {
        double val = getRawAxis(axis.key);

        // Attack 3 Y axis forwards is negative so negate in code
        if (axis == AttackThreeAxis.Y) {
            val *= -1;
        }
        
        if (Math.abs(val) <= deadband) {
            val = 0.0;
        }
        return val;
    }

    public enum AttackThreeAxis {

        /** x axis */
        X(0),
        /** y axis */
        Y(1),
        /** z axis. THIS DOESN'T EXIST BUT DRIVERSTATION SAYS IT DOES */
        Z(2);

        public final int key;

        /**
        * This is the constructor of the enumeration. The keys provided to the constructor are used to
        * access the value of each axis in getAxis().
        *
        * @param key the magical number assigned by the Driver Station
        */
        AttackThreeAxis(int key) {
            this.key = key;
        }
    }
}
