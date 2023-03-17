// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LimeLight extends SubsystemBase {
  /** Creates a new LimeLight. */
  public LimeLight() {
    
  }
  
  

  NetworkTable limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
  // Getting horizontal offset from target
  NetworkTableEntry tX = limeLightTable.getEntry("tx");
  // Getting vertical offset from target
  NetworkTableEntry tY = limeLightTable.getEntry("ty");
  /**  Getting distance from target 
   * (Read as area that the target takes up 
   * (If you still don't know remember that objects diminish with distance))
   */ 
  NetworkTableEntry tA = limeLightTable.getEntry("ta");

  NetworkTableEntry tID = limeLightTable.getEntry("tid");
  public String limeLight_currentlyViewedAprilTag;
  
  // NetworkTableEntry detectedTag = limeLightTable.getEntry("");

  // Current position of the robot on the field in (X, Y, Z)
  NetworkTableEntry botPose = limeLightTable.getEntry("botpose");

  NetworkTableEntry ledMode = limeLightTable.getEntry("ledMode");
  
  public double [] botPoseArray;

  // how many degrees the limelight is mounted from perfectly vertical
  double limelightMountAngleDegrees = 90;
  // distance from the center of the Limelight lens to the floor
  double limelightLensHeightInches = 20;
  // distance from the april tag to the floor
  double aprilTagToFloorInches = 17.12598;

  boolean tagDetected;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Getting the Limelight values from the Network tables periodically
    double x = tX.getDouble(0.0);
    double y = tY.getDouble(0.0);
    double area = tA.getDouble(0.0);
    double id = tID.getDouble(9.0);

    botPoseArray = botPose.getDoubleArray(new double[6]);
   if (id == 9) {
    tagDetected = false;
    limeLight_currentlyViewedAprilTag = "None";
   } else {
    limeLight_currentlyViewedAprilTag = Double.toString(id);
    tagDetected = true;
   }

    // Putting LimeLight values onto SmartDashboard
    SmartDashboard.putNumber("LimeLight X", botPoseArray[1]);
    SmartDashboard.putNumber("LimeLight Y", y);
    SmartDashboard.putNumber("LimeLight Area", area);
    // SmartDashboard.putString("Currently Viewed AprilTag", currentlyViewedAprilTag);
    SmartDashboard.putNumber("BotPose Z", this.GetBotPoseX());

    SmartDashboard.putBoolean("AprilTag Detected", tagDetected);
    //SmartDashboard.putString("Currently Viewed AprilTag", currentlyViewedAprilTag);
   
  }

  

  public double [] GetBotPoseArray() {
    return botPose.getDoubleArray(new double[6]);
  }

  public Translation2d GetBotPose2d() {
    return new Translation2d(botPoseArray [1], botPoseArray [2]);
  }

  public double GetBotPoseX() {
    
    return botPoseArray [1];
  }

  public double GetPOIDistance() {
    // How far offset the robot is from colinear with the apriltag (Vertical)
    double aprilTagOffsetAngle = tY.getDouble(0.0);

    double angleToAprilTagDegrees = limelightMountAngleDegrees + aprilTagOffsetAngle;
    double angleToAprilTagRadians = angleToAprilTagDegrees * (3.14159 / 180.0);

    //calculate distance
    double limelightToPOIInches = (aprilTagToFloorInches - limelightLensHeightInches)/Math.tan(angleToAprilTagRadians);

    return limelightToPOIInches;
  }

  public double GetCurrentAprilTag() {
    return tID.getDouble(9.0);
  }

  public double GetTX() {
    return tX.getDouble(42);
  }
}
