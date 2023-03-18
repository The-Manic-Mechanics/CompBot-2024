// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Solenoids;

public class DumbAuton extends CommandBase {
  private final DriveTrain sysDriveTrain;
  private final Arm sysArm;
  private final Solenoids sysSolenoids;
  private final double auton_walk_feet = 4.5;
  private boolean auton_block_placed;

  /** Creates a new DumbAuton. */
  public DumbAuton(DriveTrain inSysDriveTrain, Arm inSysArm, Solenoids inSysSolenoids) {
    sysDriveTrain = inSysDriveTrain;
    sysArm = inSysArm;
    sysSolenoids = inSysSolenoids;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysDriveTrain, sysArm, sysSolenoids);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sysDriveTrain.frontLeftEnc.reset();
    sysDriveTrain.frontRightEnc.reset();
    sysDriveTrain.backLeftEnc.reset();
    sysDriveTrain.backRightEnc.reset();
    auton_block_placed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!auton_block_placed) {
      if (sysArm.GetArmEnc() >= ArmConstants.ARM_180_DEG) {
        sysArm.SetArmSpeed(0, 0);
        sysSolenoids.ToggleClaw(Value.kForward);
        auton_block_placed = true;
      } else {
        sysArm.SetArmSpeed(1, -.40);
      }
    } else {
      double inches = auton_walk_feet * 12.0d;
      if ((sysDriveTrain.frontLeftEnc.getDistance() >= inches || 
          (sysDriveTrain.frontRightEnc.getDistance() >= inches) || 
          (sysDriveTrain.backLeftEnc.getDistance() >= inches) || 
          (sysDriveTrain.backRightEnc.getDistance() >= inches))) {
        sysDriveTrain.CartisianDrive(0, 0, 0);
      } else sysDriveTrain.CartisianDrive(-.5, 0, 0);
    }
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
