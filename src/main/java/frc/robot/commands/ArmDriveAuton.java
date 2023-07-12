// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSys;
import frc.robot.subsystems.Solenoids;

/**
* Command used in autonomous mode to drive the arm.
*/
public final class ArmDriveAuton extends CommandBase {
	/**
	* The desired arm destination as an encoder position
	*/
	private static double
	    armDestination,
	    /**
	    * The desired arm speed
	    */
		armSpeed,
		/**
		* The desired arm speed multiplier
		*/
	    armSpeedMultiplier;

	/**
	* The desired ending claw state
	*/
	private static Value clawEndStatus;

    /**
     * @param inArmEncoderDestination The desired arm destination as an encoder position
     * @param inClawEndStatus The desired ending claw state
     * @param inArmSpeed The desired arm speed
     * @param inArmSpeedMultiplier The desired arm speed multiplier
    */
	public ArmDriveAuton(ArmSys inSysArm, Solenoids inSysSolenoids, double inArmEncoderDestination, Value inClawEndStatus, double inArmSpeed, double inArmSpeedMultiplier) {
		armDestination = inArmEncoderDestination;
		clawEndStatus = inClawEndStatus;
		armSpeed = inArmSpeed;
		armSpeedMultiplier = inArmSpeedMultiplier;
		addRequirements(inSysArm, inSysSolenoids);
	}

	@Override
	public void initialize() {
		Solenoids.claw.set(Value.kOff);
	}

	@Override
	public void execute() {
		/* Sets the arm speed until it reaches or passes the armDestination */
		if (ArmSys.armEncoder.get() >= armDestination) {
			ArmSys.SetArmSpeed(0, 0);
			Solenoids.claw.set(clawEndStatus);
		} else {
			ArmSys.SetArmSpeed(armSpeed, armSpeedMultiplier);
		}
	}

	@Override
	public void end(boolean interrupted) {
		ArmSys.SetArmSpeed(0, 0);
		Solenoids.claw.set(Value.kOff);
	}

	@Override
	public boolean isFinished() {
		return clawEndStatus == Solenoids.claw.get();
	}
}
