// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DumbAuton extends CommandBase {
  private final DriveTrain sysDriveTrain;
  private double initTime = 0;
  /** Creates a new DumbAuton. */
  public DumbAuton(DriveTrain inSysDriveTrain) {
    sysDriveTrain = inSysDriveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = System.nanoTime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (initTime == -1) return;
    
    if (initTime - System.nanoTime() < 1_000_000_000 * 2.5) {
      sysDriveTrain.CartisianDrive(0, 0.7, 0);
    } else {
      sysDriveTrain.CartisianDrive(0, 0, 0);

      try {throw new Exception("Auton complete process");
      } catch (Exception e) {}
      
      initTime = -1;
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
