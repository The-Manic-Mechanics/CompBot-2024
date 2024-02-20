// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Controllers.Sax.AxisPort;
import frc.robot.Constants.Controllers.Sax.ButtonsPort;
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
    // TODO: Add encoder stops and add controller 
    Intake.driveLift(RobotContainer.saxController.getRawAxis(AxisPort.X) * Constants.Intake.LIFT_SPEED_MULTIPLIER);

    Intake.driveIntakeAuto();
    
    // Activate intake
    if (RobotContainer.saxController.getRawButton(ButtonsPort.BLUE))
      Intake.setSpeed(Constants.Intake.SPEED);
    else
      Intake.setSpeed(0);

    // Reverse intake
    if (RobotContainer.saxController.getRawButton(ButtonsPort.ORANGE))
      Intake.setSpeed(-1 * Constants.Intake.SPEED);

    // Intake position
    if (RobotContainer.saxController.getRawButton(ButtonsPort.SALMON))
      Intake.driveLiftToPos(1, Constants.Intake.LIFT_SPEED_MULTIPLIER);
    
    // Amp scoring position
    if (RobotContainer.saxController.getRawButton(ButtonsPort.YELLOW))
      Intake.driveLiftToPos(2, Constants.Intake.LIFT_SPEED_MULTIPLIER);
     
    // Shooting position
    if (RobotContainer.saxController.getRawButton(ButtonsPort.PINK))
      Intake.driveLiftToPos(3, Constants.Intake.LIFT_SPEED_MULTIPLIER);
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
