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
import frc.robot.Constants.Arm;

/**
* Stores all things pertaining to the arm (Arm motors, encoders)
*/
public final class ArmSys extends SubsystemBase {

  private static WPI_VictorSPX topArmMotor;
  private static WPI_VictorSPX bottomArmMotor;

  /**
  * Used to work two motors in sync
  */
  public static MotorControllerGroup armGroup;

  public static Encoder armEncoder;

 

  public ArmSys() {

    // Initialising Motors
    topArmMotor = new WPI_VictorSPX(Arm.Ports.TOP_ARM);
    bottomArmMotor = new WPI_VictorSPX(Arm.Ports.BOTTOM_ARM);

    topArmMotor.setInverted(true);

    armGroup = new MotorControllerGroup(topArmMotor, bottomArmMotor);

    armEncoder = new Encoder(Arm.Encoders.CHANNEL_A, Arm.Encoders.CHANNEL_B);

    // armEncoder.setDistancePerPulse(1);
    // FIXME: Could cause unwanted reset after auton
    armEncoder.reset();

    // armEncoder.setReverseDirection(true);
  }

  /**
  * Sets both arm motors to the inputted speed
   * @param speed The speed to set the motors to
   * @param speedMultiplier The percentage to cut the speed by
  */  
  public static void SetArmSpeed(double speed, double speedMultiplier) {
    assert speedMultiplier < 1;
    if (speedMultiplier == 0) {
    speedMultiplier = 1;
    }
    topArmMotor.set(speed * speedMultiplier);
    bottomArmMotor.set(speed * speedMultiplier);
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
