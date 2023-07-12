// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm.Telescoper;
import frc.robot.Constants.Arm.Claw;
import frc.robot.Constants.Arm.Brake;

/**
* Holds all solenoids' definitions
*/
public final class Solenoids extends SubsystemBase {
	public static DoubleSolenoid armTelescoper, claw, brake;

	public Solenoids() {
		// Initialising Solenoids
		armTelescoper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Telescoper.FWD_PORT, Telescoper.REVERSE_PORT);
		claw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Claw.FWD_PORT, Claw.REVERSE_PORT);
		brake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Brake.PORT_FWD, Brake.PORT_BACK);

	}
}
