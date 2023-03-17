// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DumbAuton extends CommandBase {
  private final DriveTrain sysDriveTrain;
  private Instant initTime;
  public double dumbAuton_WalkTime = 2;
  /** Creates a new DumbAuton. */
  public DumbAuton(DriveTrain inSysDriveTrain) {
    sysDriveTrain = inSysDriveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Instant.now();
    sysDriveTrain.CartisianDrive(0, 0.7, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (initTime == null) return;
    if (Duration.between(initTime, Instant.now()).toSeconds() < dumbAuton_WalkTime) {
      sysDriveTrain.CartisianDrive(0, 0.7, 0);
    } else {
      sysDriveTrain.CartisianDrive(0, 0, 0);
      initTime = null;
    }
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
