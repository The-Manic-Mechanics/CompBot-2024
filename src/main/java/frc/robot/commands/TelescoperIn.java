// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSys;
import frc.robot.subsystems.Solenoids;

/**
* What do you think?
*/
public final class TelescoperIn extends CommandBase {
	public TelescoperIn(Solenoids inSysSolenoids, ArmSys inSysArm) {
		addRequirements(inSysSolenoids, inSysArm);
	}
	
	@Override
	public void execute() {
		Solenoids.armTelescoper.set(Value.kForward);
	}
	
	@Override
	public void end(boolean interrupted) {
		Solenoids.armTelescoper.set(Value.kOff);
	}
	
	@Override
	public boolean isFinished() {
		return Solenoids.armTelescoper.get() == Value.kForward;
	}
}
