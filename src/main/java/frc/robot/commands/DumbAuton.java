// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;

public class DumbAuton extends CommandBase {
  private final DriveTrain sysDriveTrain;
  private final Arm sysArm;
  private final double auton_walk_feet = 4.5;
  private final boolean auton_block_placed = false;

  /** Creates a new DumbAuton. */
  public DumbAuton(DriveTrain inSysDriveTrain, Arm inSysArm) {
    sysDriveTrain = inSysDriveTrain;
    sysArm = inSysArm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysDriveTrain, sysArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sysDriveTrain.frontLeftEnc.reset();
    sysDriveTrain.frontRightEnc.reset();
    sysDriveTrain.backLeftEnc.reset();
    sysDriveTrain.backRightEnc.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (!auton_block_placed) {
    //   if (sysArm.GetArmEnc() >= ) {
    //     sysArm.SetArmSpeed(0, 0);
    //   } else {
    //     sysArm.SetArmSpeed(1, .40);
    //   }
    // } else {
      double inches = auton_walk_feet * 12.0d;
      if ((sysDriveTrain.frontLeftEnc.getDistance() >= inches || 
         (sysDriveTrain.frontRightEnc.getDistance() >= inches) || 
          (sysDriveTrain.backLeftEnc.getDistance() >= inches) || 
          (sysDriveTrain.backRightEnc.getDistance() >= inches))) {
        sysDriveTrain.CartisianDrive(0, 0, 0);
      } else sysDriveTrain.CartisianDrive(-.5, 0, 0);
    // }
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
