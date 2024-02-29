// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Controllers.Sax.ButtonPorts;
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
    // Hook positioner forward
    if (RobotContainer.saxController.getRawButton(ButtonPorts.PINK))
      Climber.Motors.hookPositioner.set(Constants.Climber.HOOK_POSITIONER_SPEED);
    else
      Climber.Motors.hookPositioner.set(0);

    // Raise the robot, via activating climber
    if (RobotContainer.saxController.getRawButton(ButtonPorts.JOYSTICK))
      Climber.Motors.one.set(Constants.Climber.SPEED);
    else {
      Climber.Motors.one.set(0);
      Climber.Motors.two.set(0);
    }

    // Reverse Climber
    if (RobotContainer.saxController.getRawButton(ButtonPorts.GREEN))
      Climber.Motors.one.set(-1 * Constants.Climber.SPEED);
    else {
      Climber.Motors.one.set(0);
      Climber.Motors.two.set(0);
    }

    // Hook positioner backwards
    if (RobotContainer.saxController.getRawButton(ButtonPorts.PURPLE))
      Climber.Motors.hookPositioner.set(-1 * Constants.Climber.HOOK_POSITIONER_SPEED);
    else
      Climber.Motors.hookPositioner.set(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Climber.Motors.one.set(0);
    Climber.Motors.two.set(0);

    Climber.Motors.hookPositioner.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
