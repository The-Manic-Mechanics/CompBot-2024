// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Controllers.Sax.ButtonPorts;
import frc.robot.subsystems.Shooter;

public class ShooterDrive extends Command {
  Shooter sysShooter;

  public ShooterDrive(Shooter inSysShooter) {
    sysShooter = inSysShooter;
    addRequirements(sysShooter);
  }

  @Override
  public void execute() {
    // TODO: The drivers may want this changed: Update: See class frc.robot.Controllers.
    // Turns on the shooter if it is past the threshold and if the Red button is pressed
    // if (Intake.Encoders.lift.get() <= Constants.Encoders.Intake.SHOOTER_ON_LIMIT) 
      if (RobotContainer.saxController.getRawButton(ButtonPorts.RED))
        Shooter.setSpeed(Constants.Shooter.SPEED);
      else
        Shooter.setSpeed(0);
  }

  @Override
  public void end(boolean interrupted) {
    Shooter.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
