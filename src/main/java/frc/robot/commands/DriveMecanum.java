// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

/**
* Used for driving the robot during teleop by taking in the controller values and giving them to the motors
*/
public final class DriveMecanum extends Command {
	
	DriveTrain sysDriveTrain;

	double 
		/**
		 * The robot's movement speed along the X axis (Usually stafing)
		 */
		moveSpeedX, 
		/**
		 * The robot's movement speed along the Y axis (Usually forward/backward)
		 */
		moveSpeedY, 
		/**
		 * The robot's movement speed along the Z axis (Rotation)
		 */
		moveSpeedZ,
		speedMultiplier = 1;


	public DriveMecanum(DriveTrain inSysDriveTrain) {
		sysDriveTrain = inSysDriveTrain;
		addRequirements(sysDriveTrain);
	}

	@Override
	public void execute() {
		// Get the speeds from controller and multiply it by the speed 
		moveSpeedY = speedMultiplier * RobotContainer.driverOneController.getLeftX();
		moveSpeedX = speedMultiplier * RobotContainer.driverOneController.getLeftY();
		moveSpeedZ = -speedMultiplier * RobotContainer.driverOneController.getRightX();
		// Put in controller inputs and drive the motors accordingly
		DriveTrain.mecanum.driveCartesian(moveSpeedX, moveSpeedY, moveSpeedZ);
	}

	@Override
	public void end(boolean interrupted) {
		// Set the motor speeds to zero on interupt
		DriveTrain.mecanum.driveCartesian(0, 0, 0);
	}

}
