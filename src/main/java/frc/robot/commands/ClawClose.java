// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Solenoids;

/**
 * Command that closes the claw mechanism.
 */
public final class ClawClose extends CommandBase {
	public ClawClose(Solenoids inSysSolenoids) {
		addRequirements(inSysSolenoids);
	}

	@Override
	public void execute() {
		Solenoids.claw.set(Value.kReverse);
	}

	@Override
	public void end(boolean interrupted) {
		Solenoids.claw.set(Value.kOff);
	}
	
	@Override
	public boolean isFinished() {
		return Solenoids.armTelescoper.get() == Value.kForward;
	}
}
