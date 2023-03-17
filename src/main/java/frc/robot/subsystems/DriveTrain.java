// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Collections;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.MecanumAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.DriveTrainConstants.DriveAuton;

public class DriveTrain extends SubsystemBase {
  VMXPi sysVMXPi;
  LimeLight sysLimelight;

  /** Creates a new DriveTrain. */
  MecanumDrive mecanumDrive;

  WPI_VictorSPX frontLeft;
  WPI_VictorSPX frontRight;
  WPI_VictorSPX backLeft;
  WPI_VictorSPX backRight;

  public ADXRS450_Gyro gyro;

  ChassisSpeeds mecanumChassisSpeeds;
  public MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds;

  MecanumDriveWheelPositions wheelPositions;

  public MecanumDriveKinematics mecanumDriveKinematics;

  Translation2d frontLeftLocation;
  Translation2d frontRightLocation;
  Translation2d backLeftLocation;
  Translation2d backRightLocation;

  public MecanumDriveOdometry mecanumDriveOdometry;

  Encoder frontLeftEnc;
  Encoder frontRightEnc;
  Encoder backLeftEnc;
  Encoder backRightEnc;

  public SendableChooser<Command> autoRoutineChooser;
  MecanumAutoBuilder autoBuilder;

  public DriveTrain() {

    sysLimelight = new LimeLight();
    sysVMXPi = new VMXPi();

    frontLeft = new WPI_VictorSPX(DriveTrainConstants.FRONT_LEFT_MOTOR_PORT);
    frontRight = new WPI_VictorSPX(DriveTrainConstants.FRONT_RIGHT_MOTOR_PORT);
    backLeft = new WPI_VictorSPX(DriveTrainConstants.BACK_LEFT_MOTOR_PORT);
    backRight = new WPI_VictorSPX(DriveTrainConstants.BACK_RIGHT_MOTOR_PORT);

    frontLeftLocation = new Translation2d(DriveTrainConstants.FRONT_LEFT_LOCATION,
        DriveTrainConstants.FRONT_LEFT_LOCATION);
    frontRightLocation = new Translation2d(DriveTrainConstants.FRONT_RIGHT_LOCATION,
        -1 * DriveTrainConstants.FRONT_RIGHT_LOCATION);
    backLeftLocation = new Translation2d(-1 * DriveTrainConstants.BACK_LEFT_LOCATION,
        DriveTrainConstants.BACK_LEFT_LOCATION);
    backRightLocation = new Translation2d(-1 * DriveTrainConstants.BACK_RIGHT_LOCATION,
        -1 * DriveTrainConstants.BACK_RIGHT_LOCATION);

    gyro = new ADXRS450_Gyro();

    frontLeftEnc = new Encoder(
        DriveTrainConstants.FRONT_LEFT_ENCODER_A,
        DriveTrainConstants.FRONT_LEFT_ENCODER_B);

    frontRightEnc = new Encoder(
        DriveTrainConstants.FRONT_RIGHT_ENCODER_A,
        DriveTrainConstants.FRONT_RIGHT_ENCODER_B);

    backLeftEnc = new Encoder(
        DriveTrainConstants.BACK_LEFT_ENCODER_A,
        DriveTrainConstants.BACK_LEFT_ENCODER_B);

    backRightEnc = new Encoder(
        DriveTrainConstants.BACK_RIGHT_ENCODER_A,
        DriveTrainConstants.BACK_RIGHT_ENCODER_B);

    // frontRight.setInverted(true);
    // backRight.setInverted(true);
    frontLeftEnc.setDistancePerPulse(DriveAuton.DISTANCE_PER_PULSE);
    frontRightEnc.setDistancePerPulse(DriveAuton.DISTANCE_PER_PULSE);
    backLeftEnc.setDistancePerPulse(DriveAuton.DISTANCE_PER_PULSE);
    backRightEnc.setDistancePerPulse(DriveAuton.DISTANCE_PER_PULSE);

    frontLeft.setInverted(true);
    // frontRight.setInverted(true);
    backLeft.setInverted(true);

    mecanumDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

    mecanumDriveKinematics = new MecanumDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation,
        backRightLocation);

    mecanumChassisSpeeds = new ChassisSpeeds(DriveAuton.MAX_METRES_PER_SEC, DriveAuton.MAX_METRES_PER_SEC * .5,
        DriveAuton.MAX_METRES_PER_SEC);

    mecanumDriveWheelSpeeds = mecanumDriveKinematics.toWheelSpeeds(mecanumChassisSpeeds);

    // double frontLeftSpeed = mecanumDriveWheelSpeeds.frontLeftMetersPerSecond;
    // double frontRightSpeed = mecanumDriveWheelSpeeds.frontRightMetersPerSecond;
    // double backLeftSpeed = mecanumDriveWheelSpeeds.rearLeftMetersPerSecond;
    // double backRightSpeed = mecanumDriveWheelSpeeds.rearRightMetersPerSecond;

    wheelPositions = new MecanumDriveWheelPositions(
        frontLeftEnc.getDistance(),
        frontRightEnc.getDistance(),
        backLeftEnc.getDistance(),
        backRightEnc.getDistance());

    double[] currentPose = sysLimelight.GetBotPoseArray();
    // #FIXME# Make sure all values are what you think they are in API (Like the
    // value used for rot)
    Pose2d initPose = new Pose2d(currentPose[1], currentPose[2], sysVMXPi.vmxPi.getRotation2d());

    // #TODO# Use apriltags to caculate initial pose
    mecanumDriveOdometry = new MecanumDriveOdometry(mecanumDriveKinematics, sysVMXPi.getRotation2d(), wheelPositions,
        initPose);

    // This will load the file "Example Path.path" and generate it with a max
    // velocity of 4 m/s and a max acceleration of 3 m/s^2
    // PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new
    // PathConstrains(4, 3));

    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.

    // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    // eventMap.put("intakeDown", new IntakeDown());

<<<<<<< HEAD
    MecanumAutoBuilder autoBuilder = new MecanumAutoBuilder(
      mecanumDriveOdometry :: getPoseMeters,
      this :: resetOdometry, 
      mecanumDriveKinematics, 
      new PIDConstants(.5, 0, 0), // Constants for the translation controller
      new PIDConstants(.5, 0, 0), // Constants for the rot controller
      DriveAuton.MAX_METRES_PER_SEC, 
      this :: setWheelSpeeds, 
      DriveAuton.EVENT_MAP, 
      true, 
      this
    );

    List<PathPlannerTrajectory> LinkLoadingSideBlue = PathPlanner.loadPathGroup("Loading Station Side Link", new PathConstraints(DriveAuton.MAX_METRES_PER_SEC, DriveAuton.MAX_ACCEL));
    List<PathPlannerTrajectory> LinkCommunitySideBlue = PathPlanner.loadPathGroup("Community Zone Side Link", DriveAuton.MAX_METRES_PER_SEC, DriveAuton.MAX_ACCEL);

    autoRoutineChooser = new SendableChooser<>();
    autoRoutineChooser.addOption("Link Loading Side Blue", autoBuilder.fullAuto(LinkLoadingSideBlue));
    autoRoutineChooser.addOption("Link Community Side Blue", autoBuilder.fullAuto(LinkCommunitySideBlue));
    autoRoutineChooser.setDefaultOption(getName(), null);
    SmartDashboard.putData("Auton Chooser", autoBuilder.fullAuto(LinkLoadingSideBlue));  
=======
    List<PathPlannerTrajectory> LinkLoadingSideBlue = PathPlanner.loadPathGroup("Loading Station Side Link",
        new PathConstraints(DriveAuton.MAX_METRES_PER_SEC, DriveAuton.MAX_ACCEL));
    List<PathPlannerTrajectory> LinkCommunitySideBlue = PathPlanner.loadPathGroup("Community Zone Side Link",
        DriveAuton.MAX_METRES_PER_SEC, DriveAuton.MAX_ACCEL);

    autoRoutineChooser.addOption("Link Loading Side Blue", autoBuilder.fullAuto(LinkLoadingSideBlue));
    autoRoutineChooser.addOption("Link Community Side Blue", autoBuilder.fullAuto(LinkCommunitySideBlue));
    autoRoutineChooser.setDefaultOption("Default", autoBuilder.fullAuto(LinkLoadingSideBlue));
    SmartDashboard.putData("Auton Chooser", autoRoutineChooser);

    MecanumAutoBuilder autoBuilder = new MecanumAutoBuilder(
        mecanumDriveOdometry::getPoseMeters,
        this::resetOdometry,
        mecanumDriveKinematics,
        new PIDConstants(.5, 0, 0), // Constants for the translation controller
        new PIDConstants(.5, 0, 0), // Constants for the rot controller
        DriveAuton.MAX_METRES_PER_SEC,
        this::setWheelSpeeds,
        DriveAuton.EVENT_MAP,
        true,
        this);

>>>>>>> 6695e32 (Live fixes)
  }

  public void CartisianDrive(double speedX, double speedY, double speedZ) {
    mecanumDrive.driveCartesian(speedX, speedY, speedZ);
  }

  public void MecanumDriveVolts(
      double frontLeftVolts,
      double frontRightVolts,
      double backLeftVolts,
      double backRightVolts) {
    frontLeft.setVoltage(frontLeftVolts);
    frontRight.setVoltage(frontRightVolts);
    backLeft.setVoltage(backLeftVolts);
    backRight.setVoltage(backRightVolts);
  }

  public MecanumDriveWheelSpeeds getCurMecWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        frontLeft.get(), frontRight.get(),
        backLeft.get(), backRight.get());
  }

  public MecanumDriveWheelPositions getCurMecWheelPos() {
    return new MecanumDriveWheelPositions(
        frontLeftEnc.getDistance(), frontRightEnc.getDistance(),
        backLeftEnc.getDistance(), backRightEnc.getDistance());
  }

  public void setWheelSpeeds(MecanumDriveWheelSpeeds inSpeeds) {
    frontLeft.set(inSpeeds.frontLeftMetersPerSecond);
    frontRight.set(inSpeeds.frontRightMetersPerSecond);
    backLeft.set(inSpeeds.rearLeftMetersPerSecond);
    backRight.set(inSpeeds.rearRightMetersPerSecond);
  }

  public void setDriveMotsVolts(MecanumDriveMotorVoltages inVolts) {
    frontLeft.setVoltage(inVolts.frontLeftVoltage);
    frontRight.setVoltage(inVolts.frontRightVoltage);
    backLeft.setVoltage(inVolts.rearLeftVoltage);
    backRight.setVoltage(inVolts.rearRightVoltage);
  }

  public Pose2d getPoseOd() {
    return mecanumDriveOdometry.getPoseMeters();
  }

  /* For on the fly path generation */
  public PathPlannerTrajectory genPath(
      double maxVel, double maxAccel, Translation2d transP1, double rotHead1, double rotHolo1, Translation2d transP2,
      double rotHead2, double rotHolo2) {
    PathPlannerTrajectory traj1 = PathPlanner.generatePath(

        new PathConstraints(maxVel, maxAccel),

        new PathPoint(
            transP1,
            Rotation2d.fromDegrees(rotHead1),
            Rotation2d.fromDegrees(rotHolo1)), // position, heading, holonomic rotation

        new PathPoint(
            transP2,
            Rotation2d.fromDegrees(rotHead2),
            Rotation2d.fromDegrees(rotHolo2)) // position, heading, holonomic rotation
    );

    return traj1;
  }

  public Pose2d currentAprilTag(int aimTo) {
    List<Pose2d> aprilTagCords = new ArrayList<>();
    Collections.addAll(aprilTagCords,
        new Pose2d(610.77, 42.19, new Rotation2d(3.14159)),
        new Pose2d(610.77, 108.19, new Rotation2d(3.14159)),
        new Pose2d(610.77, 174.19, new Rotation2d(3.14159)),
        new Pose2d(aimTo, aimTo, new Rotation2d(0)),
        new Pose2d(aimTo, aimTo, new Rotation2d(0)),
        new Pose2d(aimTo, aimTo, new Rotation2d(0)));
    return aprilTagCords.get(aimTo);
  }

  public void resetOdometry(Pose2d odoResetPose) {
    mecanumDriveOdometry.resetPosition(sysVMXPi.vmxPi.getRotation2d(), getCurMecWheelPos(), odoResetPose);
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    // Pose2d initHoloPose = traj.getInitialHolonomicPose();

    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            mecanumDriveOdometry.resetPosition(null, wheelPositions, mecanumDriveOdometry.getPoseMeters());
          }
        }),
        new PPMecanumControllerCommand(
            traj,
            mecanumDriveOdometry::getPoseMeters, // Pose supplier
            this.mecanumDriveKinematics, // MecanumDriveKinematics
            new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use
                                        // feedforwards.
            new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will
                                        // only use feedforwards.
            DriveAuton.MAX_METRES_PER_SEC, // Max wheel velocity meters per second
            this::setWheelSpeeds, // MecanumDriveWheelSpeeds consumer
            true, // Should the path be automatically mirrored depending on alliance color.
                  // Optional, defaults to true
            this // Requires this drive subsystem
        ));
  }

  public Command buildAuto(List<PathPlannerTrajectory> pathGroup) {
    Command out = autoBuilder.fullAuto(pathGroup);
    return out;
  }

  public FollowPathWithEvents followPathEvents(PathPlannerTrajectory inPath) {
    FollowPathWithEvents command = new FollowPathWithEvents(
        followTrajectoryCommand(inPath, false),
        inPath.getMarkers(),
        DriveAuton.EVENT_MAP);

    return command;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Rotation2d curRot2d = sysVMXPi.getRotation2d();

    mecanumDriveOdometry.update(curRot2d, wheelPositions);

    SmartDashboard.putData("frontLeft", frontLeft);
    SmartDashboard.putData("frontRight", frontRight);
    SmartDashboard.putData("backLeft", backLeft);
    SmartDashboard.putData("backRight", backRight);

    // Putting Controller Left and Right Stick Values
    SmartDashboard.putNumber("X Value", RobotContainer.driverMainController.getLeftX());
    SmartDashboard.putNumber("Y Value", RobotContainer.driverMainController.getLeftY());
    SmartDashboard.putNumber("Z Value", RobotContainer.driverMainController.getRightX());
  }
}
