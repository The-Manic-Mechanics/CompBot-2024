// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.NavX;

/**
* Class used in teleop to AutoAim to the left april tag
*/
public final class AutoAimLeft extends CommandBase {
	public AutoAimLeft(DriveTrain inSysDriveTrain, LimeLight inSysLimeLight, NavX inSysNavX) {
		addRequirements(inSysDriveTrain, inSysLimeLight, inSysNavX);
	}

	boolean isAimed;

	@Override
	public void initialize() {
		isAimed = false;
	}

	@Override
	public void execute() {
		PathPlannerTrajectory path = DriveTrain.genPath(
				1,
				.97,
				LimeLight.GetBotPose2d(),
				0,
				NavX.sensor.getAngle(),
				Constants.AprilTagCoords.TranslationFromAprilTagCoordinate(LimeLight.id, 2),
				0,
				NavX.sensor.getAngle()
		);

		DriveTrain.mecanumDriveOdometry.resetPosition(
				NavX.sensor.getRotation2d(),
				DriveTrain.getWheelPositions(),
				new Pose2d(
						LimeLight.botPoseArray[1],
						LimeLight.botPoseArray[2],
						NavX.sensor.getRotation2d()
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
