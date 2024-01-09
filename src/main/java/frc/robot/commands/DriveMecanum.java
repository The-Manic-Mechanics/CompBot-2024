// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// FIXME: This entire file needs to be reworked.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

/**
* Used for driving the robot during teleop by taking in the controller values and giving them to the motors
*/
public final class DriveMecanum extends Command {

	double speedMultThrot = 1;
	/**
	 * d = drift, ct = current time, o = old
	 */
	double 
		moveSpeedX, 
		xvar,
		moveSpeedY, 
		yvar,
		moveSpeedZ,
		zvar;


	public DriveMecanum(DriveTrain inSysDriveTrain) {
		addRequirements(inSysDriveTrain);
	}

	@Override
	public void execute() {
		xvar = RobotContainer.driverOneController.getLeftX();
		yvar = RobotContainer.driverOneController.getLeftY();
		zvar = RobotContainer.driverOneController.getRightX();
		moveSpeedY = speedMultThrot * yvar;
		moveSpeedX = -speedMultThrot * xvar;
		moveSpeedZ = -speedMultThrot * zvar;
		// Feed thhe 
		DriveTrain.mecanum.driveCartesian(moveSpeedX, moveSpeedY, moveSpeedZ);
	}

	@Override
	public void end(boolean interrupted) {
		// Stop the robot.
		DriveTrain.mecanum.driveCartesian(0, 0, 0);
	}

}
