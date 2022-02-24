package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberJumpGrabCommand extends CommandBase {

    ClimberSubsystem climberSubsystem = null;
    Timer timer;
    Double delay = 0.0;
    Boolean finished = false;

    public ClimberJumpGrabCommand(RobotContainer robotContainer) {
        // the requires(Subsystem) method must be called for each subsystem used by the command
        climberSubsystem = robotContainer.climberSubsystem;
        timer = new Timer();
    }

    /**
     * The 'initialize' method is called one time just prior to the first time
     * this Command is run after being started.
     */
    @Override
    public void initialize() {
        climberSubsystem.releaseJumperActuate();
        this.delay = climberSubsystem.getDelay();
    }

    /**
     * The 'execute' method is the main body of a command.  Is called repeatedly when
     * this * Command is scheduled to run. It is called repeatedly until either the
     * command finishes -- i.e. {@link #isFinished()}) returns true -- or it is
     * canceled.
     */
    @Override
    public void execute() {
        if (!climberSubsystem.jumperLimitSwitch.get()) {
            System.out.println("In if statement");
            Timer.delay(delay);
            climberSubsystem.clamperClose();
            climberSubsystem.jumperActuate(false);
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return finished;
    }

    /**
     * Called once when the command ended peacefully; that is it is called once
     * after {@link #isFinished}()} returns true. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the
     * command.
     */
    protected void end() {
        climberSubsystem.setStage(2);
    }
}
