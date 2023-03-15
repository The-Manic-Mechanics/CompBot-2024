// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

public class AutoAimLeft extends CommandBase {
  /** Creates a new AutoAim. */
  private final DriveTrain sysDriveTrain;
  private final LimeLight sysLimeLight;

  public AutoAimLeft(DriveTrain inSysDriveTrain, LimeLight inSysLimeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysDriveTrain = inSysDriveTrain;
    sysLimeLight = inSysLimeLight;
    addRequirements(sysDriveTrain, sysLimeLight);
  }

  double aprilTagOffsetAngle;

  double botPoseX;

  double distanceToPOI;

  boolean isAimed;

  String strafeDirection;



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isAimed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // #TODO# Instead of the code below, use trajectory generation to generate a path and follow it

    // botPoseX = sysLimeLight.GetBotPoseX();

    // if (sysLimeLight.GetCurrentAprilTag() != 9) {
    //   if (botPoseX >= -2.95) {
    //     sysDriveTrain.CartisianDrive(0, 0, -.25);

    //     strafeDirection = "r";

    //   } else if (botPoseX <= -3.05) {
    //     sysDriveTrain.CartisianDrive(0, 0, .25);

    //       strafeDirection = "l";
    //   }
    //   // #TODO# Finish This (Add Values)
    //   if ((strafeDirection == "l") && (sysLimeLight.GetTX() != 0)) {
    //     // Strafe Left
    //   } else if ((strafeDirection == "r") && (sysLimeLight.GetTX() != 0)) {
    //     // Strafe Right
    //   }

    //   double fwdDistance = sysLimeLight.GetPOIDistance();

    //   // Drive (fwdDistance) forward using encoders

    //   // if (Encoders.getDistance >= fwdDistance) {
    //   // Drive (leftDistance) left using encoders
    //   // }

    //   // if (Enoders.getDistance >= leftDistance) {
    //   isAimed = true;
    //   // }

    // } else {
    //   SmartDashboard.putString("Error", "No AprilTag Found");
    // }
    
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAimed;
  }
}
