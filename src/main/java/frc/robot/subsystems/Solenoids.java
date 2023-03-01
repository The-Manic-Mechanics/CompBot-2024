// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Solenoids extends SubsystemBase {
  private DoubleSolenoid armTelescoper;
  private DoubleSolenoid claw;
  
  /** Creates a new Solenoids. */
  public Solenoids() {
    // Initialising Solenoids
    armTelescoper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.TELESCOPER_FWD_PORT, ArmConstants.TELESCOPER_RVRSE_PORT);
    claw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.CLAW_FWD_PORT, ArmConstants.CLAW_RVRSE_PORT);
 
  }

  public void ToggleTelescope(DoubleSolenoid.Value value) {
    armTelescoper.set(value);
  }

  public void ToggleClaw(DoubleSolenoid.Value value) {
    claw.set(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
