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
  public boolean enableVision = true;

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
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    v = tv.getDouble(0.0);

    // post to smart dashboard periodically
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightV", v);
    // SmartDashboard.putNumber("LimelightArea", area);
    // SmartDashboard.putBoolean("Limelight Has Target", targetFound());
    // SmartDashboard.putNumber("Limlight degrees askew", degreesAskew());
    SmartDashboard.putNumber("Distance to Target", getDistanceToTarget());
    SmartDashboard.putBoolean("Limlight On", getVision());
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
   * @return distance to target (inches)
   */
  public double getDistanceToTarget() {
    double limelightHeight = 22.5;
    double limelightMountAngleDegrees = 28; // Angle from horizontal
    double heightToGoal = 104.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + y;
    double angleToGoalRadians = angleToGoalDegrees* (3.14159 / 180.0);

    double distance = (heightToGoal - limelightHeight) / Math.tan(angleToGoalRadians);
    return distance;

    //return (heightToGoal - limelightHeight)/Math.tan(Math.toRadians(limelightMountAngleDegrees + y));
  }

  /**
   * Turns the light on
   */
  public void lightOn() {
    ledMode.setNumber(3);
  }

  /**
   * Toggles the vision 
   */
  public void toggleVision() {
    if (!getVision()) {
      lightOn();
    } else {
      lightOff();
    }
    enableVision = getVision();
  }

  /**
   * Sets the limelight to be on or off
   * @param on true for LED on false for off
   */
  public void setVision(boolean on) {
    ledMode.setNumber((on ? 3 : 1));
  }

  /**
   * Sees if vision is enabled
   * @return True if vision is inabled else false
   */
  public boolean getVision() {
    // return enableVision;
    return ledMode.getNumber(1).intValue() == 3;
  }

  /**
   * Turns the light off
   */
  public void lightOff() {
    ledMode.setNumber(1);
  }

}
