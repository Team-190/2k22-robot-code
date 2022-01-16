// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {

  NetworkTable table;
  public NetworkTableEntry tv;
  public NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry ledMode;

  //read values periodically
  static double x;
  double y;
  double v;
  double area;

  /** Creates a new LimeLightSubsystem. */
  public LimeLightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    ledMode = table.getEntry("ledMode");
  }

  @Override
  public void periodic() {
    x = tx.getDouble(30);
    v = tv.getDouble(30);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }

  public boolean targetFound() {
    return v == 1.0;
}

  public static double degreesAskew(){
      return x;
  }

  public void lightOn() {
    ledMode.setNumber(3);
  }

  public void lightOff() {
    ledMode.setNumber(1);
  }

}
