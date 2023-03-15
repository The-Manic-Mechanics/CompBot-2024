// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

/** Creates a new GrabbyArm. */
public class Arm extends SubsystemBase {

  private WPI_VictorSPX topArmMotor;
  private WPI_VictorSPX bottomArmMotor;

  MotorControllerGroup armGroup;

  Encoder armEncoder;

 

  public Arm() {

    // Initialising Motors
    topArmMotor = new WPI_VictorSPX(ArmConstants.TOP_ARM_MOTOR_PORT);
    bottomArmMotor = new WPI_VictorSPX(ArmConstants.BOTTOM_ARM_MOTOR_PORT);

    topArmMotor.setInverted(true);

    armGroup = new MotorControllerGroup(topArmMotor, bottomArmMotor);

    armEncoder = new Encoder(ArmConstants.ARM_ENCODER_CHANNEL_A, ArmConstants.ARM_ENCODER_CHANNEL_B);

    // armEncoder.setDistancePerPulse(1);
    // #FIXME# Could cause unwanted reset after auton
    armEncoder.reset();

    // armEncoder.setReverseDirection(true);
  }

  public void SetArmSpeed(double indvSpeed, double speedMultiplier) {
    if (speedMultiplier == 0) {
    speedMultiplier = 1;
    }
    topArmMotor.set(indvSpeed * speedMultiplier);
    bottomArmMotor.set(indvSpeed * speedMultiplier);
  }

  public void SetArmGroupSpeed(double speed, double speedMultiplier) {
    // if (speedMultiplier == 0) {
    //  speedMultiplier = 1;
    // }
    armGroup.set(speed /** speedMultiplier */);
    
    
  }

  // Gets the arm encoder from the current armEncoder instance.
  public double GetArmEnc() {
    return armEncoder.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LeftStick", RobotContainer.driverSecondController.getLeftY());

    SmartDashboard.putData("TopMotor", topArmMotor);
    SmartDashboard.putData("BottomMotor", bottomArmMotor);

    SmartDashboard.putData("armEncoder", armEncoder);
    SmartDashboard.putNumber("armEncoderCounts", armEncoder.get());
  }
}
