// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

/**
* Used to Drive the robot during auton via the inputed speeds and distance
*/
public final class DriveAuton extends CommandBase {
	double driveInches, speedX, speedY, speedZ, inclineThresh;
	boolean isFinished, finishOnIncline;

    /**
     * @param inDriveInches The amount of inches to drive
     * @param inSpeedX The speed to travel in the X direction
     * @param inSpeedY The speed to travel in the Y direction
     * @param inSpeedZ The speed to travel in the Z direction
     * @param inFinishOnIncline Whether to finish when an incline is detected
     * @param inInclineThreshold The threshold for FinishOnIncline (Absolute)
    */
	public DriveAuton(
			DriveTrain inSysDriveTrain, NavX inSysNavX, double inDriveInches, double inSpeedX,
			double inSpeedY, double inSpeedZ, boolean inFinishOnIncline,
			/*Threshold as an absolute*/ double inInclineThreshold) {
		addRequirements(inSysDriveTrain, inSysNavX);
		driveInches = inDriveInches;
		speedX = inSpeedX;
		speedY = inSpeedY;
		speedZ = inSpeedZ;
		finishOnIncline = inFinishOnIncline;
		inclineThresh = inInclineThreshold;
	}

	@Override
	public void initialize() {
		isFinished = false;
	}

	@Override
	public void execute() {
		// Checking if the encoders have read the desired distance and stopping if they have
		if ((DriveTrain.Encoders.frontLeft.getDistance() >= driveInches ||
				(DriveTrain.Encoders.frontRight.getDistance() >= driveInches) ||
				(DriveTrain.Encoders.backLeft.getDistance() >= driveInches) ||
				(DriveTrain.Encoders.backRight.getDistance() >= driveInches))) {
			DriveTrain.mecanum.driveCartesian(0, 0, 0);
			isFinished = true;
		} else
			DriveTrain.mecanum.driveCartesian(speedX, speedY, speedZ);
	}


	@Override
	public void end(boolean interrupted) {
		DriveTrain.mecanum.driveCartesian(0, 0, 0);
	}

	@Override
	public boolean isFinished() {
		// TODO: Make sure Math.abs makes sense in the context of how the navX gets pitch
		// Checks if finishOnIncline is true and if the current pitch has exceeded the inclineThresh
		if (finishOnIncline && Math.abs(NavX.sensor.getPitch()) > inclineThresh) {
			return true;
		} else {
			return isFinished;
		}
	}
}
