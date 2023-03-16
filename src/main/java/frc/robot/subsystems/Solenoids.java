// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Solenoids extends SubsystemBase {
  private DoubleSolenoid armTelescoper;
  private DoubleSolenoid claw;
  private DoubleSolenoid brake;
  boolean clawState;
  
  /** Creates a new Solenoids. */
  public Solenoids() {
    // Initialising Solenoids
    armTelescoper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.TELESCOPER_FWD_PORT, ArmConstants.TELESCOPER_RVRSE_PORT);
    claw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.CLAW_FWD_PORT, ArmConstants.CLAW_RVRSE_PORT);
    brake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.BRAKING_PORT_FWD, ArmConstants.BRAKING_PORT_BACK);
 
  }

  public void ToggleBrake(DoubleSolenoid.Value value) {
    brake.set(value);
  }

  public void GetBrake(DoubleSolenoid.Value value) {
    brake.get();
  }

  public void ToggleTelescope(DoubleSolenoid.Value value) {
    armTelescoper.set(value);
  }

  public DoubleSolenoid.Value GetTelescope() {
   return armTelescoper.get();
  }

  public void ToggleClaw(DoubleSolenoid.Value value) {
    claw.set(value);
  }

  public DoubleSolenoid.Value GetClaw() {
    return claw.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (GetClaw() == Value.kForward) {
      clawState = false;
    } else {
      clawState = true;
    }
  }
}
