// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.GrabbyArm;

public class ArmDrive extends CommandBase {
  private final GrabbyArm sysArm;
  /** Creates a new ArmDrive. */
  public ArmDrive(GrabbyArm inSysArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysArm = inSysArm;

    addRequirements(sysArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = RobotContainer.driverSecondController.getLeftY();
    sysArm.SetArmSpeed(speed, 0.25);
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
