// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants.DriveAuton;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.VMXPi;

public class AutoAimFwd extends CommandBase {
  /** Creates a new AutoAim. */
  private final DriveTrain sysDriveTrain;
  private final LimeLight sysLimeLight;
  private final VMXPi sysVMXPi;

  public AutoAimFwd(DriveTrain inSysDriveTrain, LimeLight inSysLimeLight, VMXPi inSysVMXPi) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysDriveTrain = inSysDriveTrain;
    sysLimeLight = inSysLimeLight;
    sysVMXPi = inSysVMXPi;
    addRequirements(sysDriveTrain, sysLimeLight, sysVMXPi);
  }

  // double aprilTagOffsetAngle;

  // double botPoseX;

  // double distanceToPOI;

  // boolean isAimedFwd;

  // String strafeDirection;

  



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // isAimedFwd = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    sysDriveTrain.genPath(DriveAuton.MAX_METRES_PER_SEC, DriveAuton.MAX_ACCEL, sysLimeLight.GetBotPose2d(), 0, sysVMXPi.vmxPi.getAngle(), null, 0, 0);
    sysDriveTrain.mecanumDriveOdometry.resetPosition(sysVMXPi.vmxPi.getRotation2d(), sysDriveTrain.getCurMecWheelPos(), null);
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

    //   double distance = sysLimeLight.GetPOIDistance();

    //   // Drive (distance) forward using encoders

    //   // if (Encoders.getDistance >= distance) {
    //   isAimedFwd = true;
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
    return false;
  }
}