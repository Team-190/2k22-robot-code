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

  /**
   * Limelight documentation website:
   * https://docs.limelightvision.io/en/latest/networktables_api.html
   */

  NetworkTable table;
  public NetworkTableEntry tv; // Whether the limelight has any valid targets (0 or 1)
  public NetworkTableEntry tx; // Horizontal Offset From Crosshair To Target (LL2: -29.8 to 29.8 degrees)
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry ledMode;

  //read values periodically
  double x;
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
    x = -1*tx.getDouble(0);
    v = tv.getDouble(0);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightV", v);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putBoolean("Limelight Has Target", targetFound());
  }

  /**
   * Returns if limelight target is found
   * @return true if target found else false
   */
  public boolean targetFound() {
    return v == 1.0;
  }

  /**
   * Degrees the limelight is off from the target
   * @return number of degrees the limelight is off from the target
   */
  public double degreesAskew(){
      return x;
  }

  /**
   * Gets the distance to target TODO: do this
   * @return distance to target (meters)
   */
  public double getDistanceToTarget() {
    return 1;
  }

  /**
   * Turns the light on
   */
  public void lightOn() {
    ledMode.setNumber(3);
  }

  /**
   * Turns the light off
   */
  public void lightOff() {
    ledMode.setNumber(1);
  }

}