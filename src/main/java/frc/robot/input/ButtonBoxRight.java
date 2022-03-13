package frc.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonBoxRight extends Joystick {
    public JoystickButton traverseLeft,
            tilt,
            preColorWheel,
            release,
            traverseRight,
            colorWheel,
            hoodTrench;

    /**
    * Construct an instance of a joystick. The joystick index is the USB port on the drivers
    * station.,
    *
    * @param port The port on the Driver Station that the joystick is plugged into.
    */
    public ButtonBoxRight(int port) {
        super(port);

        traverseLeft = new JoystickButton(this, 2);
        tilt = new JoystickButton(this, 3);
        preColorWheel = new JoystickButton(this, 4);
        release = new JoystickButton(this, 5);
        traverseRight = new JoystickButton(this, 6);
        colorWheel = new JoystickButton(this, 7);
        hoodTrench = new JoystickButton(this, 14);
    }
}