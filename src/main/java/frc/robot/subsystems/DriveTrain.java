// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  MecanumDrive mecanumDrive;

  public DriveTrain() {

    VictorSP frontLeft = new VictorSP(DriveTrainConstants.FRONT_LEFT_MOTOR_PORT);
    VictorSP frontRight = new VictorSP(DriveTrainConstants.FRONT_RIGHT_MOTOR_PORT);
    VictorSP backLeft = new VictorSP(DriveTrainConstants.BACK_LEFT_MOTOR_PORT);
    VictorSP backRight = new VictorSP(DriveTrainConstants.BACK_RIGHT_MOTOR_PORT);

    frontRight.setInverted(true);
    frontLeft.setInverted(true);


    mecanumDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
  
  }

  public void CartisianDrive(double speedX, double speedY, double speedZ) {
    mecanumDrive.driveCartesian(speedX, speedY, speedZ);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
