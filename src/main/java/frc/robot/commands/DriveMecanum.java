// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

/**
* Used for driving the robot during teleop by taking in the controller values and giving them to the motors
*/
public final class DriveMecanum extends CommandBase {

	double speedMultThrot = 1;
	/**
	 * d = drift, ct = current time, o = old
	 */
	double moveSpeedX, xvar;
	double moveSpeedY, yvar;
	double moveSpeedZ, zvar;


	public DriveMecanum(DriveTrain inSysDriveTrain) {
		addRequirements(inSysDriveTrain);
	}

	@Override
	public void execute() {

		xvar = RobotContainer.driverMainController.getLeftX();
		yvar = RobotContainer.driverMainController.getLeftY();
		zvar = RobotContainer.driverMainController.getRightX();

        // The commented conditional statements were a shoddy solution to prevent stick drift. (deadbands)
// 		if (Math.abs(yvar) > .15) {
			moveSpeedY = speedMultThrot * yvar;
// 		} else {
// 			moveSpeedY = 0;
// 		}

// 		if (Math.abs(xvar) > .32) {
			moveSpeedX = -speedMultThrot * xvar;
//		} else {
//			moveSpeedX = 0;
//		}

// 		if (Math.abs(zvar) > .062) {
			moveSpeedZ = -speedMultThrot * zvar;
//		} else {
//			moveSpeedZ = 0;
//		}

		// The swapped variables were swapped on purpose. FIXME by hooking up the connections correctly.
		DriveTrain.mecanum.driveCartesian(moveSpeedY, moveSpeedX, moveSpeedZ);
	}

	@Override
	public void end(boolean interrupted) {
		DriveTrain.mecanum.driveCartesian(0, 0, 0);
	}

}
