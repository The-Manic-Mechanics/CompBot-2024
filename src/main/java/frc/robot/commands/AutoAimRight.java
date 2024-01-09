// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AprilTagCoords;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Gyroscope;

/**
* Command used in teleoperation to AutoAim to the right april tag
*/
public final class AutoAimRight extends CommandBase {
	public AutoAimRight(DriveTrain inSysDriveTrain, LimeLight inSysLimeLight, Gyroscope inSysGyroscope) {
		addRequirements(inSysDriveTrain, inSysLimeLight, inSysGyroscope);
	}

	boolean isAimed;
	
	@Override
	public void initialize() {
		isAimed = false;
	}
	
	@Override
	public void execute() {
		// Generates a path to be followed
		PathPlannerTrajectory path = DriveTrain.genPath(
				1,
				.97,
				LimeLight.GetBotPose2d(),
				0,
				Gyroscope.sensor.getAngle(),
				AprilTagCoords.TranslationFromAprilTagCoordinate(LimeLight.id, 3),
				0,
				Gyroscope.sensor.getAngle()
		);

		DriveTrain.mecanumDriveOdometry.resetPosition(
				Gyroscope.sensor.getRotation2d(),
				DriveTrain.getWheelPositions(),
				new Pose2d(
						LimeLight.botPoseArray[1],
						LimeLight.botPoseArray[2],
						Gyroscope.sensor.getRotation2d()
				)
		);

		// Follows var "path"
		DriveTrain.followTrajectoryCommand(path, false);

		isAimed = true;
	}
	
	@Override
	public boolean isFinished() {
		return isAimed;
	}
}
