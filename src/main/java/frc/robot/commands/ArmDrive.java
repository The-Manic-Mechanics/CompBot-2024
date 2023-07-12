// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSys;

/**
* Command used to drive the arm during teleoperation mode
*/
public final class ArmDrive extends CommandBase {
	public ArmDrive(ArmSys inSysArm) {
		addRequirements(inSysArm);
	}
	
	@Override
	public void execute() {

		double speed;

		/* Checks the Arm Encoder Position and stops the motor if it's past the limits */
		if (
				(
						ArmSys.armEncoder.get() > Arm.Limits.ARM_BACKWARDS_LIMIT
						// FIXME: Are these checks needed?
// 								&&
//						RobotContainer.driverSecondController.getLeftY() > 0
				)
						||
				(
						ArmSys.armEncoder.get() < Arm.Limits.ARM_FORWARD_LIMIT
// 								&&
//						RobotContainer.driverSecondController.getLeftY() < 0
				)
		) {

			speed = 0;
		} else {
			speed = -1 * RobotContainer.driverSecondController.getLeftY();
		}

		ArmSys.SetArmSpeed(speed, Arm.Speeds.MULTIPLIER);
	}
}
