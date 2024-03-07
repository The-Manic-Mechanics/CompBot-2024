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

  public ClimberDrive(Climber inSysClimber) {
    sysClimber = inSysClimber;
    addRequirements(sysClimber);
  }

  @Override
  public void execute() {
    // Following are immediate switches for different button>action bindings.

    // Move the hook positioner forwards.
    if (RobotContainer.saxController.getRawButton(ButtonPorts.PINK))
      Climber.Motors.hookPositioner.set(Constants.Climber.HOOK_POSITIONER_SPEED);
    else
      Climber.Motors.hookPositioner.set(0);

    // Move the climber upwards.
    if (RobotContainer.saxController.getRawButton(ButtonPorts.JOYSTICK))
      Climber.Motors.one.set(Constants.Climber.SPEED);
    else {
      Climber.Motors.one.set(0);
      Climber.Motors.two.set(0);
    }

    // Move the climber downwards.
    if (RobotContainer.saxController.getRawButton(ButtonPorts.GREEN))
      Climber.Motors.one.set(-1 * Constants.Climber.SPEED);
    else {
      Climber.Motors.one.set(0);
      Climber.Motors.two.set(0);
    }

    // Move the hook positioner backwards.
    if (RobotContainer.saxController.getRawButton(ButtonPorts.PURPLE))
      Climber.Motors.hookPositioner.set(-1 * Constants.Climber.HOOK_POSITIONER_SPEED);
    else
      Climber.Motors.hookPositioner.set(0);
  }

  @Override
  public void end(boolean interrupted) {
    Climber.Motors.one.set(0);
    Climber.Motors.two.set(0);
    Climber.Motors.hookPositioner.set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
