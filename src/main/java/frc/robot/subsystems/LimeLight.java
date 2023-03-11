// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  String currentlyViewedAprilTag;
  
  // NetworkTableEntry detectedTag = limeLightTable.getEntry("");

  // Current position of the robot on the field in (X, Y, Z)
  NetworkTableEntry botPose = limeLightTable.getEntry("botpose");

  NetworkTableEntry ledMode = limeLightTable.getEntry("ledMode");
  


  // how many degrees the limelight is mounted from perfectly vertical
  double limelightMountAngleDegrees = 90;
  // distance from the center of the Limelight lens to the floor
  double limelightLensHeightInches = 20;
  // distance from the april tag to the floor
  double aprilTagToFloorInches = 17.12598;

 


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Getting the Limelight values from the Network tables periodically
    double x = tX.getDouble(0.0);
    double y = tY.getDouble(0.0);
    double area = tA.getDouble(0.0);
    double id = tID.getDouble(9.0);

    if (id == 1) {
    currentlyViewedAprilTag = "1";
      } else if (id == 2) {
      currentlyViewedAprilTag = "2";
        } else if (id == 3) {
        currentlyViewedAprilTag = "3";
          } else if (id == 4) {
          currentlyViewedAprilTag = "4";
            } else if (id == 5) {
            currentlyViewedAprilTag = "5";
              } else if (id == 6) {
              currentlyViewedAprilTag = "6";
                } else if (id == 7) {
                currentlyViewedAprilTag = "7";
                  } else if (id == 8) {
                  currentlyViewedAprilTag = "8";
                    } else if (id == 9) {
                    currentlyViewedAprilTag = "None";
                      }
    
   // botPose.getDoubleArray(new double [6]);

    // Putting LimeLight values onto SmartDashboard
    SmartDashboard.putNumber("LimeLight X", x);
    SmartDashboard.putNumber("LimeLight Y", y);
    SmartDashboard.putNumber("LimeLight Area", area);
    SmartDashboard.putString("Currently Viewed AprilTag", currentlyViewedAprilTag);
    SmartDashboard.putNumber("BotPose Z", this.GetBotPoseX());
   
  }

  public double [] GetBotPose() {
    return botPose.getDoubleArray(new double[6]);
  }

  public double GetBotPoseX() {
    double [] botPoseArray = botPose.getDoubleArray(new double[6]);
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
