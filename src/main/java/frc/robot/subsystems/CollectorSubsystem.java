// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

public class CollectorSubsystem extends SubsystemBase {
  //TODO: fix channels once robot is wired
  public final DoubleSolenoid collectorPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, CollectorConstants.FORWARD_CHANNEL, CollectorConstants.REVERSE_CHANNEL);
  public final CANSparkMax upperBallPathMotor = new CANSparkMax(CollectorConstants.UPPERBALLPATH_CHANNEL, CANSparkMaxLowLevel.MotorType.kBrushed);
  public final CANSparkMax collectorMotor = new CANSparkMax(CollectorConstants.COLLECTOR_CHANNEL, CANSparkMaxLowLevel.MotorType.kBrushless);
  public final DigitalInput photoElectricSensor = new DigitalInput(CollectorConstants.PHOTOELECTRIC_SENSOR_ID);

  private boolean toggle = false;
  
  /** Creates a new CollectorSubsystem. */
  public CollectorSubsystem() {
    // collectorMotor.setSmartCurrentLimit(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Detect Ball", detectBallpath());
    SmartDashboard.putBoolean("Get Toggle", getToggle());
  }

  public void collect(double speed){
    collectorMotor.set(speed);
  }

  public void retract(){
    collectorPiston.set(Value.kReverse);
  }

  public boolean detectBallpath() {
    return !photoElectricSensor.get();
  }

  /**
    * Get the current for the Collector Motor
    *
    * @return the current as a double
    */
    public double getCurrent() {
      return collectorMotor.getOutputCurrent();
  }

  public void extend(){
    collectorPiston.set(Value.kForward);
  }

  public void toggleCollector(double speed) {
    toggle = !toggle;
    if (toggle) {
      collect(speed);
      extend();

    } else {
      collect(0);
      retract();
    }

  }

  public boolean getToggle() {
    return toggle;
  }

  public void toggle(){
    collectorPiston.toggle();
  }

  public void upperBallPath(double speed){
    upperBallPathMotor.set(speed);
  }

}
