// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveMecanum extends CommandBase {
  /** Creates a new DriveMecanum. */
  private final DriveTrain sysDriveTrain;

  double moveSpeedY;
  double moveSpeedX;
  double moveSpeedZ;

  public DriveMecanum(DriveTrain inSysDriveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysDriveTrain = inSysDriveTrain;

    addRequirements(sysDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    // if (RobotContainer.driverMainController.getRightTriggerAxis() != 0) {
    //   moveSpeedX = RobotContainer.driverMainController.getRightTriggerAxis();
    // }

    if (Math.abs(RobotContainer.driverMainController.getY()) > .082) {
      moveSpeedY = RobotContainer.driverMainController.getY();
    } else {
      moveSpeedY = 0;
    }

    if (Math.abs(RobotContainer.driverMainController.getX()) > .045) {
      moveSpeedX = -1 * RobotContainer.driverMainController.getX();
    } else {
      moveSpeedX = 0;
    }
    
    if (Math.abs(RobotContainer.driverMainController.getX()) > .062)
    moveSpeedZ = -1  * RobotContainer.driverMainController.getX();
    
    sysDriveTrain.CartisianDrive(moveSpeedY, moveSpeedX, moveSpeedZ);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysDriveTrain.CartisianDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
