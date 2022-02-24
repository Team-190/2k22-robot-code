package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    Solenoid jumper = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.JUMPER_ID);
    Solenoid release_jumper = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.RELEASE_JUMPER_ID);
    Solenoid clamper = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.CLAMPER_ID);
    Solenoid pivot = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.PIVOT_ID);
    Solenoid extender = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.EXTENDER_ID);
    int jumpStage = 0;

    public DigitalInput jumperLimitSwitch = new DigitalInput(0); // Limit switch is pressed when in neutral state,
                                                                 // not pressed when "jumping"

    public AnalogInput distanceSensor = new AnalogInput(0); // Ultrasonic Sensor

    WPI_TalonFX climber_motor = new WPI_TalonFX(ClimberConstants.CLIMBER_MOTOR_CHANNEL);

    // Choosers
    private final SendableChooser<Double> delayTimeChooser = new SendableChooser<>();

    public ClimberSubsystem() {

        for (double i = 0.01; i < 0.5; i+=0.01) {
            delayTimeChooser.addOption(i+" Seconds", i);
        }

        SmartDashboard.putData(delayTimeChooser);
    }

    /**
     * Turns the jumper solenoid on or off
     * @param on state of the solenoid
     */
    public void jumperActuate(boolean on) {
        jumper.set(on);
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

    public void clamperClose() {
        clamper.set(true);
    }

    public void clamperOpen() {
        clamper.set(false);
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

    public Double getDelay() {
        return delayTimeChooser.getSelected();
    }

    /**
     * gets current climbing stage
     * @return an integer representing the current climbing stage
     */
    public int getStage(){
        return jumpStage;
    }

    /**
     * sets climbing stage to stage
     * @param stage the stage we're setting to
     */
    public void setStage(int stage){
        jumpStage = stage;
    }

    /**
     * Gets the distance of the robot from the ground
     * @return The distance of the robot from the ground (in inches)
     */
    public double getDistance() {
        double rawValue = distanceSensor.getValue();
        // voltage_scale_factor allows us to compensate for differences in supply voltage.
        double voltage_scale_factor = 5/RobotController.getVoltage5V();
        double currentDistanceInches = rawValue * voltage_scale_factor * 0.0492;
        return currentDistanceInches;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Jumper Limit Switch", jumperLimitSwitch.get());
    }


}
