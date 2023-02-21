// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  MecanumDrive mecanumDrive;

  WPI_VictorSPX frontLeft;
  WPI_VictorSPX frontRight;
  WPI_VictorSPX backLeft;
  WPI_VictorSPX backRight;
 

  public DriveTrain() {

    frontLeft = new WPI_VictorSPX(DriveTrainConstants.FRONT_LEFT_MOTOR_PORT);
    frontRight = new WPI_VictorSPX(DriveTrainConstants.FRONT_RIGHT_MOTOR_PORT);
    backLeft = new WPI_VictorSPX(DriveTrainConstants.BACK_LEFT_MOTOR_PORT);
    backRight = new WPI_VictorSPX(DriveTrainConstants.BACK_RIGHT_MOTOR_PORT);

    // frontRight.setInverted(true);
    // backRight.setInverted(true);

    frontLeft.setInverted(true);
    // frontRight.setInverted(true);
    backLeft.setInverted(true);


    mecanumDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
  
  }

  public void CartisianDrive(double speedX, double speedY, double speedZ) {
    mecanumDrive.driveCartesian(speedX, speedY, speedZ);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData("frontLeft", frontLeft);
    SmartDashboard.putData("frontRight", frontRight);
    SmartDashboard.putData("backLeft", backLeft);
    SmartDashboard.putData("backRight", backRight);
  }
}
