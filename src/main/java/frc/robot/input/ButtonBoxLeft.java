package frc.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonBoxLeft extends Joystick {
    public JoystickButton slideIn,
            slideOut,
            spinLeft,
            spinRight,
            hood,
            collect,
            unJam,
            shoot,
            preClimb,
            climb,
            reverseBallPath;

    /**
    * Construct an instance of a joystick. The joystick index is the USB port on the drivers
    * station.,
    *
    * @param port The port on the Driver Station that the joystick is plugged into.
    */
    public ButtonBoxLeft(int port) {
        super(port);

        slideIn = new JoystickButton(this, 2);
        slideOut = new JoystickButton(this, 1);
        spinLeft = new JoystickButton(this, 3);
        hood = new JoystickButton(this, 4);
        spinRight = new JoystickButton(this, 5);
        collect = new JoystickButton(this, 6);
        unJam = new JoystickButton(this, 7);
        preClimb = new JoystickButton(this, 8);
        climb = new JoystickButton(this, 9);
        shoot = new JoystickButton(this, 10);
        reverseBallPath = new JoystickButton(this, 11);
    }
}