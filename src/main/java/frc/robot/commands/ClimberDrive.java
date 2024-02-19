// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Controllers.Sax.ButtonsPort;
import frc.robot.subsystems.Climber;


public class ClimberDrive extends Command {
  Climber sysClimber;
  /** Creates a new ClimberDrive. */
  public ClimberDrive(Climber inSysClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysClimber = inSysClimber;
    addRequirements(sysClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.saxController.getRawButton(ButtonsPort.SALMON))
      Climber.Motors.one.set(Constants.Climber.HOOK_POSITIONER_SPEED);
    else
      Climber.Motors.one.set(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Climber.Motors.one.set(0);

    Climber.Motors.hookPositioner.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
