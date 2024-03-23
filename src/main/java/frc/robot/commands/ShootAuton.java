// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Used to shoot a note.
 */
public final class ShootAuton extends Command {
	boolean isFinished;

	public ShootAuton() {
	}

	@Override
	public void initialize() {
		isFinished = false;
	}

	@Override
	public void execute() {
		Shooter.setSpeed(1);
		Timer.delay(2);
		Intake.setSpeed(-frc.robot.Constants.Intake.SPEED);
		Timer.delay(.8);
		Shooter.setSpeed(0);
		Intake.setSpeed(0);
		isFinished = true;
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}
}
