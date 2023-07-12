// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under t``he terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Solenoids;
import frc.robot.subsystems.ArmSys;
import frc.robot.subsystems.NavX;

// TODO: Turn some of these params into constants? (addCommands params params)
/**
* Sequential command for autonomous that places a piece and auto-balances
*/
public final class AutoBalanceAuton extends SequentialCommandGroup {
	public AutoBalanceAuton(DriveTrain inSysDriveTrain, NavX inSysNavX, Solenoids inSysSolenoids, ArmSys inSysArm) {
		addCommands(
				new ArmDriveAuton(inSysArm, inSysSolenoids, Arm.Limits.POS_180_DEG, Value.kForward, 1, -.40),
				new ArmDriveAuton(inSysArm, inSysSolenoids, 900, Value.kOff, 1, .40),
				new DriveAuton(inSysDriveTrain, inSysNavX, 5000d, -.6, 0d, 0d, true, 5),
				new AutoBalance(inSysNavX, inSysDriveTrain, true, 5),
				new BrakeUp(inSysSolenoids)
		);
	}
}