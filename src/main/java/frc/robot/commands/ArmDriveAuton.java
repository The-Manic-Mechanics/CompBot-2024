// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Solenoids;

public class ArmDriveAuton extends CommandBase {
  private final Arm sysArm;
  private final Solenoids sysSolenoids;
  double armDestination, clawSpeed, clawSpeedMultiplier;
  Value clawOpenSetStatus;
  /** Creates a new ArmDriveAuton. */
  public ArmDriveAuton(Arm inSysArm, Solenoids inSysSolenoids, double inArmEncoderDestination, Value inClawOpenSetStatus, double inClawSpeed, double inClawSpeedMultiplier) {
    sysArm = inSysArm;
    sysSolenoids = inSysSolenoids;
    armDestination = inArmEncoderDestination;
    clawOpenSetStatus = inClawOpenSetStatus;
    clawSpeed = inClawSpeed;
    clawSpeedMultiplier = inClawSpeedMultiplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysArm, sysSolenoids);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sysSolenoids.ToggleClaw(Value.kOff);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (sysArm.GetArmEnc() >= armDestination) {
      sysArm.SetArmSpeed(0, 0);
      sysSolenoids.ToggleClaw(clawOpenSetStatus);
    } else {
      sysArm.SetArmSpeed(clawSpeed, clawSpeedMultiplier);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysArm.SetArmSpeed(0, 0);
    sysSolenoids.ToggleClaw(Value.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (clawOpenSetStatus == sysSolenoids.GetClaw() )
    return true;
    else 
    return false;
  }
}
