// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Auton;
import frc.robot.Constants.PIDControllers.Holonomic;
import frc.robot.Constants.PIDControllers.WheelVelocities;
import frc.robot.subsystems.ComplexAuton;
import frc.robot.subsystems.DriveTrain;

/**
 * Command to generate a trajectory to the best and nearest shooting position
 * and then have the physical robot follow said Trajectory via a scheduled
 * Command.
 */
public class AutoShooterAlign extends Command {
  DriveTrain sysDriveTrain;
  ComplexAuton sysComplexAuton;

  MecanumControllerCommand mecanumController;

  CommandScheduler scheduler;

  public static boolean isAligning;

  double sideLenOne;
  double sideLenTwo;

  double least;
  double prevLeast;

  int closestPointIndex;

  public AutoShooterAlign(DriveTrain inSysDriveTrain, ComplexAuton inSysComplexAuton) {
    sysDriveTrain = inSysDriveTrain;
    sysComplexAuton = inSysComplexAuton;

    addRequirements(sysDriveTrain, sysComplexAuton);
  }

  @Override
  public void initialize() {
    scheduler = CommandScheduler.getInstance();

    double[] distances = new double[Constants.Shooter.SHOOTING_POSITIONS.length];

    // Looping through SHOOTING_POSITIONS[] and storing the distance of each BotPose into distances[].
    for (int i = 0; i != Constants.Shooter.SHOOTING_POSITIONS.length; i++) {
      sideLenOne = Math.abs(Constants.Shooter.SHOOTING_POSITIONS[i].getX() - ComplexAuton.getPoseDual().get().getX());
      sideLenTwo = Math.abs(Constants.Shooter.SHOOTING_POSITIONS[i].getY() - ComplexAuton.getPoseDual().get().getY());

      distances[i] = Math.sqrt(Math.pow(sideLenOne, 2) + Math.pow(sideLenTwo, 2));
    }

    least = distances[0];

    // Stores the index of the least value in distances[] in closestPointIndex.
    for (int i = 1; i != distances.length; i++) {
      prevLeast = least;
      least = Math.min(least, distances[i]);

      if (least == distances[0])
        closestPointIndex = 0;
      else if (least != prevLeast)
        closestPointIndex = i;
    }

    mecanumController = new MecanumControllerCommand(
        TrajectoryGenerator.generateTrajectory(
            Arrays.asList(ComplexAuton.getPoseDual().get(), Constants.Shooter.SHOOTING_POSITIONS[closestPointIndex]),
            new TrajectoryConfig(
                Auton.MAX_SPEED,
                Auton.MAX_ACCEL)),
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

    scheduler.schedule(mecanumController);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveTrain.mecanum.driveCartesian(0, 0, 0);

    scheduler.cancel(mecanumController);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((RobotContainer.driverOneController.getLeftX() != 0)
        ||
        (RobotContainer.driverOneController.getLeftY() != 0)
        ||
        (RobotContainer.driverOneController.getRightX() != 0)) {
      isAligning = false;
      return true;
    } else if (mecanumController.isFinished()) {
      isAligning = false;
      return true;
    } else {
      isAligning = true;
      return false;
    }
  }
}
