package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.DrivetrainConstants;

public class ClimberSubsystem extends SubsystemBase {

    Solenoid jumper = new Solenoid(ClimberConstants.JUMPER_ID);
    Solenoid release_jumper = new Solenoid(ClimberConstants.RELEASE_JUMPER_ID);
    Solenoid clamper = new Solenoid(ClimberConstants.CLAMPER_ID);
    Solenoid pivot = new Solenoid(ClimberConstants.PIVOT_ID);
    Solenoid extender = new Solenoid(ClimberConstants.EXTENDER_ID);

    WPI_TalonFX climber_motor = new WPI_TalonFX(ClimberConstants.CLIMBER_MOTOR_CHANNEL);

    public ClimberSubsystem() {

        
    }

    /** 
    *
    *
    *
    *
    */
    public void jumperActuate() {
        jumper.set(true);
    }

    public void releaseJumperActuate() {
        release_jumper.set(true);
    }

    public void clamperToggle() {
        clamper.toggle();
    }

    public void pivotActuate() {
        pivot.set(true);
    }

    public void extenderActuate() {
        extender.set(true);
    }

    @Override
    public void periodic() {
    }


}
