// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HumanInterface;
import frc.robot.subsystems.Intake;

public class IntakeDrive extends Command {
  Intake sysIntake;

  public IntakeDrive(Intake inSysIntake) {
    sysIntake = inSysIntake;
    addRequirements(sysIntake);
  }

  @Override
  public void execute() {
    Intake.driveLift(HumanInterface.IntakeDrive.getLiftDriveAxis() * frc.robot.Constants.Intake.LIFT_SPEED_MULTIPLIER);
    Intake.driveIntakeAuto();
    
    // Following are immediate switches for different button>action bindings.
    
    // Drive the intake mechanism backwards.
    if (HumanInterface.IntakeDrive.inDesired() /* TODO: && Intake.Encoders.lift.get() <= Constants.Encoders.Intake.ON_LIMIT */)
      Intake.setSpeed(-1 * frc.robot.Constants.Intake.SPEED);
    else if (HumanInterface.IntakeDrive.outDesired())
    // Drive the intake mechanism forwards.
      Intake.setSpeed(frc.robot.Constants.Intake.SPEED);
    else
      Intake.setSpeed(0);

    // Drive the lift to the intake position.
    // if (RobotContainer.saxController.getRawButton(Bindings.))
    //   Intake.driveLiftToPos(1, frc.robot.Constants.Intake.LIFT_SPEED_MULTIPLIER);
    // else if (RobotContainer.saxController.getRawButton(ButtonPorts.YELLOW))
    // Drive the lift to the shooting position.
      // Intake.driveLiftToPos(3, frc.robot.Constants.Intake.LIFT_SPEED_MULTIPLIER);
  }

  @Override
  public void end(boolean interrupted) {
    Intake.setSpeed(0);
    Intake.Motors.lift.set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
