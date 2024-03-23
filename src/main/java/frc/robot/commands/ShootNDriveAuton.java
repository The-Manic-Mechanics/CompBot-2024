// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Used to shoot a note and then drive the robot during auton via the inputted speeds and distance.
 */
public final class ShootNDriveAuton extends Command {
	double driveInches;
	double speedX;
	double speedY;
	double speedZ;
	double oldPosition;
	boolean isFinished;
	boolean throwComplete;

	/**
	 * @param inDriveInches The amount of inches to drive.
	 * @param inSpeedX The speed to travel in the X direction.
	 * @param inSpeedY The speed to travel in the Y direction.
	 * @param inSpeedZ The speed to travel in the Z direction.
	 */
	public ShootNDriveAuton(
			DriveTrain inSysDriveTrain, 
			Gyroscope inSysGyroscope, 
			double inDriveInches, 
			double inSpeedX,
			double inSpeedY, 
			double inSpeedZ
	) {
		addRequirements(inSysDriveTrain, inSysGyroscope);
		driveInches = inDriveInches;
		speedX = inSpeedX;
		speedY = inSpeedY;
		speedZ = inSpeedZ;
	}

	@Override
	public void initialize() {
		isFinished = throwComplete = false;
		DriveTrain.Encoders.frontLeft.setPosition(0);
		DriveTrain.Encoders.frontRight.setPosition(0);
		DriveTrain.Encoders.rearLeft.setPosition(0);
		DriveTrain.Encoders.rearRight.setPosition(0);
	}

	@Override
	public void execute() {
		if (!throwComplete) {
			Shooter.setSpeed(1);
			Timer.delay(2);
			Intake.setSpeed(-frc.robot.Constants.Intake.SPEED);
			Timer.delay(.8);
			Shooter.setSpeed(0);
			Intake.setSpeed(0);
			throwComplete = true;
		}
		SmartDashboard.putNumber("c_DriveInches", driveInches);
		SmartDashboard.putNumber("c_encFrontLeft", DriveTrain.Encoders.frontLeft.getPosition());
		// Checking if the encoders have read the desired distance, if so stop.
		if (
			Math.abs(DriveTrain.Encoders.frontLeft.getPosition()) >= driveInches
			||
			Math.abs(DriveTrain.Encoders.frontRight.getPosition()) >= driveInches
			||
			Math.abs(DriveTrain.Encoders.rearLeft.getPosition()) >= driveInches
			||
			Math.abs(DriveTrain.Encoders.rearRight.getPosition()) >= driveInches
		) {
			DriveTrain.mecanum.driveCartesian(0, 0, 0);
			isFinished = true;
		} else
			DriveTrain.mecanum.driveCartesian(speedX, speedY, speedZ);
	}

	@Override
	public void end(boolean interrupted) {
		// Stop the robot.
		DriveTrain.mecanum.driveCartesian(0, 0, 0);
		// Reset encoders.
		DriveTrain.Encoders.frontLeft.setPosition(0);
		DriveTrain.Encoders.frontRight.setPosition(0);
		DriveTrain.Encoders.rearLeft.setPosition(0);
		DriveTrain.Encoders.rearRight.setPosition(0);
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}
}
