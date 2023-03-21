// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Solenoids;

public class BrakeDown extends CommandBase {
  Solenoids sysSolenoids;
  /** Creates a new AutoDrive. */
  public BrakeDown(Solenoids inSysSolenoids) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysSolenoids = inSysSolenoids;
    addRequirements(sysSolenoids);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sysSolenoids.ToggleBrake(Value.kForward);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysSolenoids.ToggleBrake(Value.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (sysSolenoids.GetBrake() == Value.kForward) {
      return true;
    }
    return false;
  }
}
