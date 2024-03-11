// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Auton;
import frc.robot.commands.AutoShooterAlign;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.Constants.PIDControllers.*;

public class RobotContainer {
  private static Gyroscope sysGyroscope = new Gyroscope();
  private static DriveTrain sysDriveTrain = new DriveTrain();
  private static ComplexAuton sysComplexAuton = new ComplexAuton();

  public static AutoShooterAlign cmdAutoShooterAlign = new AutoShooterAlign(sysDriveTrain, sysComplexAuton);

  public static Pose2d initPose;

  public RobotContainer() {
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

    configureBindings();
  }

  // TODO: Bind SysID commands to gamepad buttons.
  private void configureBindings() {
    // Define the controls used in new functions within HumanInterface.CommandMap and then supply the command here.
    // Example: HumanInterface.CommandMap.straightAuton(cmdStraightAuton);
    HumanInterface.CommandMap.autoShooterAlign(cmdAutoShooterAlign);
  }

  public Command getAutonomousCommand() {
    // If the "no trajectory" option is chosen, do not run anything.
    if (DriveTrain.Odometry.autonPathChooser.getSelected() == null)
      return new InstantCommand(() -> {});
    MecanumControllerCommand mecanumController = new MecanumControllerCommand(
        // Get the selected path from Shuffleboard
        DriveTrain.Odometry.autonPathChooser.getSelected(),
        // Pose supplier
        ComplexAuton.getPoseDual(),
        ComplexAuton.feedforward,
        DriveTrain.Kinematics.mecanumDriveKinematics,
        // X controller (Field Space)
        new PIDController(Holonomic.XCONTROLLER_P, Holonomic.XCONTROLLER_I, Holonomic.XCONTROLLER_D),
        // Y controller (Field Space)
        new PIDController(Holonomic.YCONTROLLER_P, Holonomic.YCONTROLLER_I, Holonomic.YCONTROLLER_D),
        // Rotation controller
        new ProfiledPIDController(Holonomic.THETACONTROLLER_P, Holonomic.THETACONTROLLER_I, Holonomic.THETACONTROLLER_D,
            new TrapezoidProfile.Constraints(Auton.MAX_SPEED, Auton.MAX_ACCEL)),
        Auton.MAX_SPEED,
        // FrontLeft speed controller
        new PIDController(WheelVelocities.FL_CONTROLLER_P, WheelVelocities.FL_CONTROLLER_I,
            WheelVelocities.FL_CONTROLLER_D),
        // RearLeft speed controller
        new PIDController(WheelVelocities.RL_CONTROLLER_P, WheelVelocities.RL_CONTROLLER_I,
            WheelVelocities.RL_CONTROLLER_D),
        // FrontRight speed controller
        new PIDController(WheelVelocities.FR_CONTROLLER_P, WheelVelocities.FR_CONTROLLER_I,
            WheelVelocities.FR_CONTROLLER_D),
        // RearRight speed controller
        new PIDController(WheelVelocities.RR_CONTROLLER_P, WheelVelocities.RR_CONTROLLER_I,
            WheelVelocities.RR_CONTROLLER_D),
        // Wheelspeeds supplier
        DriveTrain.Kinematics.getWheelSpeeds(),
        // Voltage consumer that drives the robot using inputted voltage
        DriveTrain.Kinematics::driveVolts,
        sysDriveTrain,
        sysComplexAuton);

    return Commands.sequence(
        new InstantCommand(() -> {
          DriveTrain.Odometry.resetDriveOdometry(DriveTrain.Odometry.autonPathChooser.getSelected().getInitialPose());
        }),
        mecanumController,
        new InstantCommand(() -> {
          DriveTrain.mecanum.driveCartesian(0, 0, 0);
        }));
  }
}
