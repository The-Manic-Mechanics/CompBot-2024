// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

/** Creates a new GrabbyArm. */
public class GrabbyArm extends SubsystemBase {


  private WPI_VictorSPX topArmMotor;
  private WPI_VictorSPX bottomArmMotor;

  MotorControllerGroup armGroup;

  private DoubleSolenoid armTelescoper;
  private DoubleSolenoid claw;

  public GrabbyArm() {

    // Initialising Solenoids
    armTelescoper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.TELESCOPER_FWD_PORT, ArmConstants.TELESCOPER_RVRSE_PORT);
    claw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.CLAW_FWD_PORT, ArmConstants.CLAW_RVRSE_PORT);

    // #TODO# Set Ports To CAN ids
    // Initialising Motors
    topArmMotor = new WPI_VictorSPX(ArmConstants.TOP_ARM_MOTOR_PORT);
    bottomArmMotor = new WPI_VictorSPX(ArmConstants.BOTTOM_ARM_MOTOR_PORT);

    topArmMotor.setInverted(true);

    armGroup = new MotorControllerGroup(topArmMotor, bottomArmMotor);

  }

  public void SetArmSpeed(double speed, double speedMultiplier) {
    if (speedMultiplier == 0) {
      speedMultiplier = 1;
    }
    armGroup.set(speed * speedMultiplier);
    
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
