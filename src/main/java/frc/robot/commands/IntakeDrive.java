// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Controllers.Sax.AxisPort;
import frc.robot.Constants.Controllers.Sax.ButtonPorts;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeDrive extends Command {
  Intake sysIntake;
  /** Creates a new IntakeDrive. */
  public IntakeDrive(Intake inSysIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysIntake = inSysIntake;
    addRequirements(sysIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Intake.driveLift(RobotContainer.saxController.getRawAxis(AxisPort.X) * frc.robot.Constants.Intake.LIFT_SPEED_MULTIPLIER);

    Intake.driveIntakeAuto();
    
    // Activate intake
    if (RobotContainer.saxController.getRawButton(ButtonPorts.BLUE))
      Intake.setSpeed(frc.robot.Constants.Intake.SPEED);
    else if (Intake.Encoders.lift.get() <= Constants.Encoders.Intake.ON_LIMIT)
      Intake.setSpeed(0);

    // Reverse intake
    if (RobotContainer.saxController.getRawButton(ButtonPorts.ORANGE))
      Intake.setSpeed(-1 * frc.robot.Constants.Intake.SPEED);
    else
      Intake.setSpeed(0);

    // Intake position
    if (RobotContainer.saxController.getRawButton(ButtonPorts.SALMON))
      Intake.driveLiftToPos(1, frc.robot.Constants.Intake.LIFT_SPEED_MULTIPLIER);
    
    // Shooting position
    if (RobotContainer.saxController.getRawButton(ButtonPorts.YELLOW))
      Intake.driveLiftToPos(3, frc.robot.Constants.Intake.LIFT_SPEED_MULTIPLIER);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.setSpeed(0);
    Intake.Motors.lift.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
