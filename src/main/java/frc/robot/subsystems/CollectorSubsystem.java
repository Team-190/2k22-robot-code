package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

public class CollectorSubsystem extends SubsystemBase {

    // Motor controller CAN Ids
    private final WPI_TalonSRX collectorMotor =
            new WPI_TalonSRX(CollectorConstants.COLLECTOR_MOTOR_CHANNEL);

    // Solenoid Pins
    private final DoubleSolenoid collectorSolenoid =
            new DoubleSolenoid(
                    PneumaticsModuleType.CTREPCM,
                    CollectorConstants.COLLECTOR_SOLENOID_PORT_IN,
                    CollectorConstants.COLLECTOR_SOLENOID_PORT_OUT);

    public enum CollectorState {
        DEPLOYED,
        UNDEPLOYED
    }

    /** Constructor for Collector Subsystem */
    public CollectorSubsystem() {}

    /** Set the Collector Motor to intake into the Chaos Revolver */
    public void intake() {
        collectorMotor.set(0.5);
    }

    /** Set the Collector Motor to expunge, so that we can unjam from the roller bar */
    public void outtake() {
        collectorMotor.set(-0.75);
    }

    /** Stop the Collector Motor */
    public void stop() {
        collectorMotor.set(0);
    }

    /** Deploy the collector with the Pistons */
    public void deploy() {
        collectorSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    /** Retract the collector with the Pistons */
    public void undeploy() {
        collectorSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    /**
    * Get the current for the Collector Motor
    *
    * @return the current as a double
    */
    public double getCurrent() {
        return collectorMotor.getStatorCurrent();
    }

    /**
    * Get the current state of the Collector by reading the solenoid
    *
    * @return The Collector State, either Deployed or Undeployed
    */
    public CollectorState getState() {
        CollectorState currentState;

        if (collectorSolenoid.get() == DoubleSolenoid.Value.kForward) {
            currentState = CollectorState.DEPLOYED;
        } else {
            currentState = CollectorState.UNDEPLOYED;
        }

        return currentState;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Collector Current", getCurrent());
    }
}
