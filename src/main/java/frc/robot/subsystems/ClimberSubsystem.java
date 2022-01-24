package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DrivetrainConstants;

public class ClimberSubsystem extends SubsystemBase {

    Solenoid jumper = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.JUMPER_ID);
    Solenoid release_jumper = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.RELEASE_JUMPER_ID);
    Solenoid clamper = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.CLAMPER_ID);
    Solenoid pivot = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.PIVOT_ID);
    Solenoid extender = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.EXTENDER_ID);

    WPI_TalonFX climber_motor = new WPI_TalonFX(ClimberConstants.CLIMBER_MOTOR_CHANNEL);

    public ClimberSubsystem() {

        
    }

    /**
     * Turns the jumper solenoid on
     */
    public void jumperActuate() {
        jumper.set(true);
    }

    /**
     * Turns the release jumper solenoid on
     */
    public void releaseJumperActuate() {
        release_jumper.set(true);
    }

    /**
     * Toggles the clamper solenoid on and off
     */
    public void clamperToggle() {
        clamper.toggle();
    }

    /**
     * Turns the pivot solenoid on
     */
    public void pivotActuate() {
        pivot.set(true);
    }

    /**
     * Turns the extender solenoid on
     */
    public void extenderActuate() {
        extender.set(true);
    }

    /**
     * Extends the climber
     * @param speed the speed of the climber motor [-1, 1]
     */
    public void extendClimber(double speed) {
        climber_motor.set(speed);
    }

    /**
     * Determines if the climber extender has reached the forward limit
     * @return true if it has reached the limit, false otherwise
     */
    public boolean climberLimitFwd() {
        return climber_motor.isFwdLimitSwitchClosed() == 1;
    }

    /**
     * Determines if the climber extender has reached the reverse limit
     * @return true if it has reached the limit, false otherwise
     */
    public boolean climberLimitRev() {
        return climber_motor.isRevLimitSwitchClosed() == 1;
    }

    @Override
    public void periodic() {
    }


}
