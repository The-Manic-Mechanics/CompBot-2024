// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under t``he terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Solenoids;
import frc.robot.subsystems.VMXPi;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalanceAuton extends SequentialCommandGroup {
  private final DriveTrain sysDriveTrain;
  private final VMXPi sysVMXPi;
  private final Solenoids sysSolenoids;
  private final Arm sysArm;
  /** Creates a new AutoBalanceAuton. */
  public AutoBalanceAuton(DriveTrain inSysDriveTrain, VMXPi inSysVMXPi, Solenoids inSysSolenoids, Arm inSysArm) {
    sysDriveTrain = inSysDriveTrain;
    sysVMXPi = inSysVMXPi;
    sysSolenoids = inSysSolenoids;
    sysArm = inSysArm;
    addCommands(
      new ArmDriveAuton(sysArm, sysSolenoids, ArmConstants.ARM_180_DEG, Value.kForward, 1, -.40),
      new ArmDriveAuton(sysArm, sysSolenoids, 900, Value.kOff, 1, .40),
      new DriveAuton(sysDriveTrain, sysVMXPi, 5000d, -.6, 0d, 0d, true, 5),
      new AutoBalance(sysVMXPi, sysDriveTrain, true, 5),
      new BrakeUp(inSysSolenoids)
    );
  }
}