package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    // Initialize Solenoids
    Solenoid leftPivot = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.LEFT_PIVOT_ID);
    Solenoid rightPivot = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.RIGHT_PIVOT_ID);
    Solenoid leftBrake = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.LEFT_BRAKE_ID);
    Solenoid rightBrake = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.RIGHT_BRAKE_ID);

    // Initailize Motors
    WPI_TalonFX climber_motor = new WPI_TalonFX(ClimberConstants.CLIMBER_MOTOR_CHANNEL);

    boolean togglePivot = false;


    public ClimberSubsystem() {

        climber_motor.configFactoryDefault();

        // kF calculation found here: https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html
        configPIDF(
                    ClimberConstants.CLIMBER_P, 
                    ClimberConstants.CLIMBER_I, 
                    ClimberConstants.CLIMBER_D, 
                    1023 / rpmToTicksPer100ms(ClimberConstants.MAX_MOTOR_RPM));

        climber_motor.configClosedLoopPeakOutput(0, 1);
        climber_motor.configClosedLoopPeriod(0, 1);
        climber_motor.configClosedloopRamp(0.00);
        climber_motor.configAllowableClosedloopError(0, ClimberConstants.CLIMBER_TOLERANCE);

        configFeedbackSensors();

        climber_motor.setNeutralMode(NeutralMode.Brake);
        climber_motor.setInverted(true);

    }

    private void configFeedbackSensors() {
        climber_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, ClimberConstants.PID_LOOPTYPE, ClimberConstants.TIMEOUT_MS);
    }

    public void configPIDF(double P, double I, double D, double F) {
        climber_motor.config_kP(ClimberConstants.SLOT_ID, P);
        climber_motor.config_kI(ClimberConstants.SLOT_ID, I);
        climber_motor.config_kD(ClimberConstants.SLOT_ID, D);
        climber_motor.config_kF(ClimberConstants.SLOT_ID, F);
      }

    /**
     * Converts rotations per minute to ticks per 100ms
     * Multiplies rotations per minute by ticks per rotation
     * Divides by 600 to convert from minute to millisecond
     * @param rpm input of rotations per minute
     * @return output of ticks per 100ms
     */
    public double rpmToTicksPer100ms(double rpm) {
        return rpm * ClimberConstants.TICKS_PER_ROTATION / 600;
    }
    
    /**
     * Turns the left brake solenoid on or off
     * @param on True for unbrake False for brake
     */
    public void leftBrakeActuate(boolean on) {
        leftBrake.set(on);
    }

    /**
     * Turns the right brake solenoid on or off
     * @param on True for unbrake False for brake
     */
    public void rightBrakeActuate(boolean on) {
        rightBrake.set(on);
    }

    /**
     * Turns the left pivot solenoid on or off
     * @param on True for extended else false
     */
    public void leftPivotActuate(boolean on) {
        leftPivot.set(on);
    }

    /**
     * Turns the right pivot solenoid on or off
     * @param on True for extended else false
     */
    public void rightPivotActuate(boolean on) {
        rightPivot.set(on);
    }

    /**
     * Extends the climber
     * @param speed the speed of the climber motor [-1, 1]
     */
    public void extendClimber(double speed) {
        climber_motor.set(speed);
    }

    /**
     * Extend the climber to a set position
     * @param setpoint position for the climber to go to (in ticks)
     */
    public void climberPID(double setpoint){
        /*
        if (setpoint > ShooterConstants.HOOD_MAXIMUM_LIMIT) {
        degrees = ShooterConstants.HOOD_MAXIMUM_LIMIT;
        } else if (degrees < ShooterConstants.HOOD_MINIMUM_LIMIT) {
        degrees = ShooterConstants.HOOD_MINIMUM_LIMIT;
        }
        */


        climber_motor.configMotionCruiseVelocity(rpmToTicksPer100ms(ClimberConstants.CLIMBER_MOTOR_VELOCITY));
        climber_motor.configMotionAcceleration(rpmToTicksPer100ms(ClimberConstants.CLIMBER_MOTOR_ACCELERATION));
        climber_motor.configMotionSCurveStrength(ClimberConstants.CLIMBER_MOTOR_MOTION_SMOOTHING);

        climber_motor.set(ControlMode.MotionMagic, setpoint);
    }

    /**
     * Move the climber relatively by the setpoint
     * @param setpoint the number of tickcs to relatively move by
     */
    public void relativeClimberPID(double setpoint) {
        climberPID(climber_motor.getClosedLoopTarget() + setpoint);
    }

    /**
     * Checks to see if the motion is complete
     * @return true if complete else false
     */
    public boolean isMotionComplete() {
        return (Math.abs(climber_motor.getSelectedSensorPosition() - climber_motor.getClosedLoopTarget()) < ClimberConstants.CLIMBER_TOLERANCE);
    }

    /**
     * Gets the position of the climber
     * @return The encoder position of the climber (in ticks)
     */
    public double getClimberPosition() {
        return climber_motor.getSelectedSensorPosition();
    }

    /**
     * Toggles both pivots for the climber out and in
     */
    public void togglePivot() {
        togglePivot = !togglePivot;
        leftPivotActuate(togglePivot);
        rightPivotActuate(togglePivot);
    }

    /**
     * Gets the state of the toggle pivot
     * @return true for pivot out false for pivot in
     */
    public boolean getTogglePivot() {
        return togglePivot;
    }

    public void resetClimberPos() {
        climber_motor.setSelectedSensorPosition(0);
    }
    

    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("Jumper Limit Switch", jumperLimitSwitch.get());
        SmartDashboard.putNumber("Climber Position", climber_motor.getSelectedSensorPosition());
    }


}
