// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Auton;
import frc.robot.Constants.Controllers;
import frc.robot.commands.ClimberDrive;
import frc.robot.commands.DriveMecanum;
import frc.robot.commands.IntakeDrive;
import frc.robot.commands.ShooterDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ComplexAuton;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import org.ejml.data.CMatrixRMaj;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PIDControllers.*;

public class RobotContainer {
  public static SendableChooser<Command> autoRoutineChooser;
  public static SendableChooser<Trajectory> autonPathChooser;
  private final Gyroscope sysGyroscope = new Gyroscope();
  public static final XboxController driverOneController = new XboxController(Controllers.DRIVERONE_PORT);
  public static final XboxController driverTwoController = new XboxController(Controllers.DRIVERTWO_PORT);

  public static final GenericHID saxController = new GenericHID(Constants.Controllers.Sax.PORT);
  public static final DriveTrain sysDriveTrain;
  public static final ComplexAuton sysComplexAuton;

  public RobotContainer() {
    sysDriveTrain = new DriveTrain();
    sysComplexAuton = new ComplexAuton();
    Intake sysIntake = new Intake();
    Shooter sysShooter = new Shooter();
    Climber sysClimber = new Climber();

    ShooterDrive cmdShooterDrive = new ShooterDrive(sysShooter);
    IntakeDrive cmdIntakeDrive = new IntakeDrive(sysIntake);
    DriveMecanum cmdDriveMecanum = new DriveMecanum(sysDriveTrain);
    ClimberDrive cmdClimberDrive = new ClimberDrive(sysClimber);

    sysShooter.setDefaultCommand(cmdShooterDrive);
    sysDriveTrain.setDefaultCommand(cmdDriveMecanum);
    sysIntake.setDefaultCommand(cmdIntakeDrive);
    sysClimber.setDefaultCommand(cmdClimberDrive);

    autoRoutineChooser = new SendableChooser<>();
    autonPathChooser = new SendableChooser<Trajectory>();

    SmartDashboard.putData("Auton Chooser", autoRoutineChooser);
  }

  public Command getAutonomousCommand() {
    MecanumControllerCommand mecanumController = new MecanumControllerCommand(
        autonPathChooser.getSelected(),
        ComplexAuton.getPoseDual(),
        ComplexAuton.feedforward,
        DriveTrain.Kinematics.mecanumDriveKinematics,
        // X controller (Field Space)
        new PIDController(Holonomic.XCONTROLLER_P, Holonomic.XCONTROLLER_I, Holonomic.XCONTROLLER_D),
        // Y controller (Field Space)
        new PIDController(Holonomic.YCONTROLLER_P, Holonomic.YCONTROLLER_I, Holonomic.YCONTROLLER_D),
        new ProfiledPIDController(Holonomic.THETACONTROLLER_P, Holonomic.THETACONTROLLER_I, Holonomic.THETACONTROLLER_D,
            new TrapezoidProfile.Constraints(Auton.MAX_SPEED, Auton.MAX_ACCEL)),
        Auton.MAX_SPEED,
        new PIDController(WheelVelocities.FL_CONTROLLER_P, WheelVelocities.FL_CONTROLLER_I,
            WheelVelocities.FL_CONTROLLER_D),
        new PIDController(WheelVelocities.RL_CONTROLLER_P, WheelVelocities.RL_CONTROLLER_I,
            WheelVelocities.RL_CONTROLLER_D),
        new PIDController(WheelVelocities.FR_CONTROLLER_P, WheelVelocities.FR_CONTROLLER_I,
            WheelVelocities.FR_CONTROLLER_D),
        new PIDController(WheelVelocities.RR_CONTROLLER_P, WheelVelocities.RR_CONTROLLER_I,
            WheelVelocities.RR_CONTROLLER_D),
        DriveTrain.Kinematics.getWheelSpeeds(),
        DriveTrain.Kinematics::driveVolts,
        sysDriveTrain,
        sysComplexAuton);

    return Commands.sequence(
        new InstantCommand(() -> {
          DriveTrain.Odometry.resetDriveOdometry(autonPathChooser.getSelected().getInitialPose());
          DriveTrain.Motors.frontLeft.setSafetyEnabled(false);
          DriveTrain.Motors.frontRight.setSafetyEnabled(false);
          DriveTrain.Motors.rearLeft.setSafetyEnabled(false);
          DriveTrain.Motors.rearRight.setSafetyEnabled(false);
        }),
        mecanumController,
        new InstantCommand(() -> {
          DriveTrain.mecanum.driveCartesian(0, 0, 0);
          DriveTrain.Motors.frontLeft.setSafetyEnabled(true);
          DriveTrain.Motors.frontRight.setSafetyEnabled(true);
          DriveTrain.Motors.rearLeft.setSafetyEnabled(true);
          DriveTrain.Motors.rearRight.setSafetyEnabled(true);
        }));
  }
}
