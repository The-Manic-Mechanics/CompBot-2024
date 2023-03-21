// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VMXPi;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalanceAuton extends SequentialCommandGroup {
  private final DriveTrain sysDriveTrain;
  private final VMXPi sysVMXPi;
  /** Creates a new AutoBalanceAuton. */
  public AutoBalanceAuton(DriveTrain inSysDriveTrain, VMXPi inSysVMXPi) {
    sysDriveTrain = inSysDriveTrain;
    sysVMXPi = inSysVMXPi;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      /*#TODO# Add values */
    new DriveAuton(sysDriveTrain, sysVMXPi, 0d, -.5, 0d, 0d, true, 0d),
    new AutoBalance(sysVMXPi, sysDriveTrain, true, 0d)
    );
  }
}
