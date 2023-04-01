// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmDrive extends CommandBase {
  private final Arm sysArm;
  private double armSpeedMultiplier = 0.6;
  // private final Solenoids sysSolenoids;
  /** Creates a new ArmDrive. */
  public ArmDrive(Arm inSysArm/*, Solenoids inSysSolenoids*/) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysArm = inSysArm;
    // sysSolenoids = inSysSolenoids;

    addRequirements(sysArm/* ,  sysSolenoids */);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed;

    if ((sysArm.GetArmEnc() > ArmConstants.MAX_ARM_BACK_ROT &&
         RobotContainer.driverSecondController.getLeftY() > 0) ||
         (sysArm.GetArmEnc() < 400 &&
         RobotContainer.driverSecondController.getLeftY() < 0)) {
          
          speed = 0;
    } else {
      speed = -1 * RobotContainer.driverSecondController.getLeftY();
    }

    
    
    sysArm.SetArmSpeed(speed, armSpeedMultiplier * ArmConstants.ARM_SPEED_MUL_MUL);
    // sysArm.SetArmSpeed(speed, 0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
