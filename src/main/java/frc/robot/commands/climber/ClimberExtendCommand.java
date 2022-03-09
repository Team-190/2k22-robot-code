package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberExtendCommand extends Command {

    ClimberSubsystem climberSubsystem = null;

    public ClimberExtendCommand(RobotContainer robotContainer) {
        // the requires(Subsystem) method must be called for each subsystem used by the command

        climberSubsystem = robotContainer.climberSubsystem;

    }

    /**
     * The 'initialize' method is called one time just prior to the first time
     * this Command is run after being started.
     */
    @Override
    protected void initialize() {

    }

    /**
     * The 'execute' method is the main body of a command.  Is called repeatedly when
     * this * Command is scheduled to run. It is called repeatedly until either the
     * command finishes -- i.e. {@link #isFinished()}) returns true -- or it is
     * canceled.
     */
    @Override
    protected void execute() {

        climberSubsystem.extendClimber(0.5);

    }

    /**
     * <p>
     * Returns whether this command is finished. If it is, then the command will be removed and
     * {@link #end}()} will be called.
     * </p><p>
     * It may be useful for a team to reference the {@link #isTimedOut}()}
     * method for time-sensitive commands.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Returning true will result in the
     * command executing once and finishing immediately. It is recommended to use
     * {@link edu.wpi.first.wpilibj.command.InstantCommand} (added in 2017) for this.
     * </p>
     *
     * @return whether this command is finished.
     * @see Command#isTimedOut}() isTimedOut()
     */
    @Override
    protected boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return climberSubsystem.climberLimitFwd();
    }

    /**
     * Called once when the command ended peacefully; that is it is called once
     * after {@link #isFinished}()} returns true. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the
     * command.
     */
    @Override
    protected void end() {
        climberSubsystem.extendClimber(0);
        climberSubsystem.setStage(3);
    }

    /**
     * <p>
     * Called when the command ends because somebody called {@link #cancel()} or
     * another command shared the same requirements as this one, and booted it out. For example,
     * it is called when another command that requires one or more of the same
     * subsystems is scheduled to run.
     * </p><p>
     * This is where you may want to wrap up loose ends, like shutting off a motor
     * used in the command.
     * </p><p>
     * Generally, it is useful to simply call the {@link #end}()} method within this
     * method, as done here.
     * </p>
     */
    @Override
    protected void interrupted() {

    }
}