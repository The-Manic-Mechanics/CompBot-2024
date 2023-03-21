// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VMXPi;

public class DriveAuton extends CommandBase {
  private final DriveTrain sysDriveTrain;
  private final VMXPi sysVMXPi;

  double driveInches;

  double speedX;
  double speedY;
  double speedZ;

  boolean isFinished;
  boolean finishOnOffset;
  
  double offsetThresh;
  /** Creates a new DriveAuton. */
  public DriveAuton(
    DriveTrain inSysDriveTrain, VMXPi inSysVMXPi,  double inDriveInches, 
    double inSpeedX, double inSpeedY, double inSpeedZ, 
    boolean inFinishOnOffset,/*Threshold as an absolute*/ double inOffsetThresh
    ) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysDriveTrain = inSysDriveTrain;
    sysVMXPi = inSysVMXPi;

    driveInches = inDriveInches;

    speedX = inSpeedX;
    speedY = inSpeedY;
    speedZ = inSpeedZ;

    finishOnOffset = inFinishOnOffset;

    offsetThresh = inOffsetThresh;

    addRequirements(sysDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((sysDriveTrain.frontLeftEnc.getDistance() >= driveInches || 
    (sysDriveTrain.frontRightEnc.getDistance() >= driveInches) || 
    (sysDriveTrain.backLeftEnc.getDistance() >= driveInches) || 
    (sysDriveTrain.backRightEnc.getDistance() >= driveInches))) {
      sysDriveTrain.CartisianDrive(0, 0, 0);
      isFinished = true;
  } else sysDriveTrain.CartisianDrive(speedX, speedY, speedZ);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysDriveTrain.CartisianDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // #TODO# Make sure Math.abs makes sense in the context of how the navX gets roll
    if (finishOnOffset && Math.abs(sysVMXPi.vmxPi.getRoll()) > offsetThresh) {
      return true;
    } else {
      return isFinished;
    }
  }
}
