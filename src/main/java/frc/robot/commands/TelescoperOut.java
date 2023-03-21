// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Solenoids;

public class TelescoperOut extends CommandBase {
  /** Creates a new DriveMecanum. */
  // #TODO# Figure out how to combine Arm and Solenoids subsystems into one for command usage
  private final Solenoids sysSolenoids;
  private final Arm sysArm;
  boolean finished;

  public TelescoperOut(Solenoids inSysSolenoids, Arm inSysArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysSolenoids = inSysSolenoids;
    sysArm = inSysArm;

    addRequirements(sysSolenoids, sysArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //  if ((sysArm.GetArmEnc() > 5500 ) || (sysArm.GetArmEnc() < 450 ) 
    //  && 
    //  RobotContainer.driverSecondController.getYButtonPressed()) {
    //   // sysSolenoids.ToggleTelescope(Value.kOff);
    //   finished = true;
    //  } else {
      if ((sysArm.GetArmEnc() > 5500 ) || (sysArm.GetArmEnc() < 100 )) {
      sysSolenoids.ToggleTelescope(Value.kReverse);
      }
    //   finished = true;
    //  }

    if (sysSolenoids.GetTelescope() == Value.kReverse) {
      finished = true;
    } else {
      finished = false;
    }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysSolenoids.ToggleTelescope(Value.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
