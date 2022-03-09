// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CollectorConstants;

public class CollectorSubsystem extends SubsystemBase {
  //TODO: fix channels once robot is wired
  public final DoubleSolenoid collectorPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  public final CANSparkMax upperBallPathMotor = new CANSparkMax(CollectorConstants.UPPERBALLPATH_CHANNEL, CANSparkMaxLowLevel.MotorType.kBrushed);
  public final CANSparkMax collectorMotor = new CANSparkMax(CollectorConstants.COLLECTOR_CHANNEL, CANSparkMaxLowLevel.MotorType.kBrushless);
  /** Creates a new CollectorSubsystem. */
  public CollectorSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void collect(double speed){
    collectorMotor.set(speed);
  }

  public void retract(){
    collectorPiston.set(Value.kReverse);
  }

  public void extend(){
    collectorPiston.set(Value.kForward);
  }

  public void toggle(){
    collectorPiston.toggle();
  }

  public void upperBallPath(double speed){
    upperBallPathMotor.set(speed);
  }

}
