// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {
  VMXPi sysVMXPi;
  LimeLight sysLimelight;

  /** Creates a new DriveTrain. */
  MecanumDrive mecanumDrive;

  WPI_VictorSPX frontLeft;
  WPI_VictorSPX frontRight;
  WPI_VictorSPX backLeft;
  WPI_VictorSPX backRight;

  ADXRS450_Gyro gyro;

  ChassisSpeeds mecanumChassisSpeeds;
  MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds;

  MecanumDriveWheelPositions wheelPositions;

  MecanumDriveKinematics mecanumDriveKinematics;

  Translation2d frontLeftLocation;
  Translation2d frontRightLocation;
  Translation2d backLeftLocation;
  Translation2d backRightLocation;

  MecanumDriveOdometry mecanumDriveOdometry;

  Encoder frontLeftEnc;
  Encoder frontRightEnc;
  Encoder backLeftEnc;
  Encoder backRightEnc;

  public DriveTrain() {

    frontLeft = new WPI_VictorSPX(DriveTrainConstants.FRONT_LEFT_MOTOR_PORT);
    frontRight = new WPI_VictorSPX(DriveTrainConstants.FRONT_RIGHT_MOTOR_PORT);
    backLeft = new WPI_VictorSPX(DriveTrainConstants.BACK_LEFT_MOTOR_PORT);
    backRight = new WPI_VictorSPX(DriveTrainConstants.BACK_RIGHT_MOTOR_PORT);

    gyro = new ADXRS450_Gyro();

    frontLeftEnc = new Encoder(
      DriveTrainConstants.FRONT_LEFT_ENCODER_A,
      DriveTrainConstants.FRONT_LEFT_ENCODER_B
    );

    frontRightEnc = new Encoder(
      DriveTrainConstants.FRONT_RIGHT_ENCODER_A,
      DriveTrainConstants.FRONT_RIGHT_ENCODER_B
    );

    backLeftEnc = new Encoder(
      DriveTrainConstants.BACK_LEFT_ENCODER_A,
      DriveTrainConstants.BACK_LEFT_ENCODER_B
    );

    backRightEnc = new Encoder(
      DriveTrainConstants.BACK_RIGHT_ENCODER_A,
      DriveTrainConstants.BACK_RIGHT_ENCODER_B
    );

    // frontRight.setInverted(true);
    // backRight.setInverted(true);

    frontLeft.setInverted(true);
    // frontRight.setInverted(true);
    backLeft.setInverted(true);


    mecanumDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

    mecanumChassisSpeeds = new ChassisSpeeds(3.87096, 1.93548, 3.87096);

    mecanumDriveWheelSpeeds = mecanumDriveKinematics.toWheelSpeeds(mecanumChassisSpeeds);

    frontLeftLocation = new Translation2d(DriveTrainConstants.FRONT_LEFT_LOCATION, DriveTrainConstants.FRONT_LEFT_LOCATION);
    frontRightLocation = new Translation2d(DriveTrainConstants.FRONT_RIGHT_LOCATION, -1 * DriveTrainConstants.FRONT_RIGHT_LOCATION);
    backLeftLocation = new Translation2d(-1 * DriveTrainConstants.BACK_LEFT_LOCATION, DriveTrainConstants.BACK_LEFT_LOCATION);
    backRightLocation = new Translation2d(-1 * DriveTrainConstants.BACK_RIGHT_LOCATION, -1 * DriveTrainConstants.BACK_RIGHT_LOCATION);

    mecanumDriveKinematics = new MecanumDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    double frontLeftSpeed = mecanumDriveWheelSpeeds.frontLeftMetersPerSecond;
    double frontRightSpeed = mecanumDriveWheelSpeeds.frontRightMetersPerSecond;
    double backLeftSpeed = mecanumDriveWheelSpeeds.rearLeftMetersPerSecond;
    double backRightSpeed = mecanumDriveWheelSpeeds.rearRightMetersPerSecond;

    wheelPositions = new MecanumDriveWheelPositions(
      frontLeftEnc.getDistance(), 
      frontRightEnc.getDistance(), 
      backLeftEnc.getDistance(), 
      backRightEnc.getDistance()
    );

    double [] currentPose = sysLimelight.GetBotPose();
    // #FIXME# Make sure all values are what you think they are in API (Like the value used for rot)
    Rotation2d rot = new Rotation2d(currentPose[3]);

    Pose2d fieldPose = new Pose2d(currentPose[1], currentPose[2], rot);

    // #TODO# Use apriltags to caculate initial pose
    mecanumDriveOdometry = new MecanumDriveOdometry(mecanumDriveKinematics,  sysVMXPi.getRotation2d(), wheelPositions, null);

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

    // Putting Controller Left and Right Stick Values
    SmartDashboard.putNumber("LeftStickY Value", RobotContainer.driverMainController.getLeftY());
    SmartDashboard.putNumber("RightStickY Value", RobotContainer.driverMainController.getRightY());
  }
}
