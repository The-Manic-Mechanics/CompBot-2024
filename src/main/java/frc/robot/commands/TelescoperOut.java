// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabbyArm;

public class TelescoperOut extends CommandBase {
  /** Creates a new DriveMecanum. */
  private final GrabbyArm sysArm;

  public TelescoperOut(GrabbyArm inSysArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysArm = inSysArm;

    addRequirements(sysArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sysArm.ToggleTelescope(Value.kForward);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysArm.ToggleTelescope(Value.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
