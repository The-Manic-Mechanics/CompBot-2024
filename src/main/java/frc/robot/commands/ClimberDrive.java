// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.HumanInterface;
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
    if (HumanInterface.ClimberDrive.hookForwardDesired())
      Climber.Motors.hookPositioner.set(Constants.Climber.HOOK_POSITIONER_SPEED);
    // Move the hook positioner backwards.
    else if (HumanInterface.ClimberDrive.hookBackwardDesired())
      Climber.Motors.hookPositioner.set(-1 * Constants.Climber.HOOK_POSITIONER_SPEED);
    else
      Climber.Motors.hookPositioner.set(0);

    // Drive the climber.
    if (HumanInterface.ClimberDrive.climberDriveDesired())
      Climber.Motors.one.set(Constants.Climber.SPEED);
    // Reverse the climber.
    else if (HumanInterface.ClimberDrive.climberReverseDesired())
      Climber.Motors.one.set(-1 * Constants.Climber.SPEED);
    else {
      Climber.Motors.one.set(0);
      Climber.Motors.two.set(0);
    }
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
