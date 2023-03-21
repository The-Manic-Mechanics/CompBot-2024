// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveAuton extends CommandBase {
  private final DriveTrain sysDriveTrain;
  double driveInches;
  double speedX;
  double speedY;
  double speedZ;
  boolean isFinished;
  /** Creates a new DriveAuton. */
  public DriveAuton(DriveTrain inSysDriveTrain, double inDriveInches, double inSpeedX, double inSpeedY, double inSpeedZ) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysDriveTrain = inSysDriveTrain;
    driveInches = inDriveInches;
    speedX = inSpeedX;
    speedY = inSpeedY;
    speedZ = inSpeedZ;

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
    return isFinished;
  }
}
