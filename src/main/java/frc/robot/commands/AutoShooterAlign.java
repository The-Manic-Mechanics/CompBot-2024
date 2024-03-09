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
import frc.robot.HumanInterface;
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

  private double sideLenOne;
  private double sideLenTwo;

  private double least;
  private double prevLeast;

  int closestPointIndex;

  public AutoShooterAlign(DriveTrain inSysDriveTrain, ComplexAuton inSysComplexAuton) {
    sysDriveTrain = inSysDriveTrain;
    sysComplexAuton = inSysComplexAuton;

    addRequirements(sysDriveTrain, sysComplexAuton);
  }

  @Override
  public void initialize() {
    scheduler = CommandScheduler.getInstance();

    /**
     * Stores the distances from the robot to each shooting position in the array in the order of the shooting positions array
     */
    double[] distances = new double[Constants.Shooter.SHOOTING_POSITIONS.length];

    // Looping through SHOOTING_POSITIONS[] and storing the distance of each BotPose into distances[].
    for (int i = 0; i != Constants.Shooter.SHOOTING_POSITIONS.length; i++) {
      sideLenOne = Math.abs(Constants.Shooter.SHOOTING_POSITIONS[i].getX() - ComplexAuton.getPoseDual().get().getX());
      sideLenTwo = Math.abs(Constants.Shooter.SHOOTING_POSITIONS[i].getY() - ComplexAuton.getPoseDual().get().getY());

      distances[i] = Math.sqrt(Math.pow(sideLenOne, 2) + Math.pow(sideLenTwo, 2));
    }

    least = distances[0];

    // Looping through distances[] finding the smallest value and storing said value's index in closestPointIndex
    for (int i = 1; i != distances.length; i++) {
      prevLeast = least;
      least = Math.min(least, distances[i]);

      if (least == distances[0])
        closestPointIndex = 0;
      else if (least != prevLeast)
        closestPointIndex = i;
    }

    mecanumController = new MecanumControllerCommand(
        // Generating a Trajectory from the robot's current pose to the closest shooting position
        TrajectoryGenerator.generateTrajectory(
            Arrays.asList(ComplexAuton.getPoseDual().get(), Constants.Shooter.SHOOTING_POSITIONS[closestPointIndex]),
            new TrajectoryConfig(
                Auton.MAX_SPEED,
                Auton.MAX_ACCEL)),
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
        // Voltage consumer to drive the robot using voltage
        DriveTrain.Kinematics::driveVolts,
        sysDriveTrain,
        sysComplexAuton);

    scheduler.schedule(mecanumController);
  }

  @Override
  public void end(boolean interrupted) {
    scheduler.cancel(mecanumController);
    DriveTrain.mecanum.driveCartesian(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    // If any of the controller axies change, stop the pathfollowing command
    if ((HumanInterface.DriveMecanum.getAxisX() != 0)
        ||
        (HumanInterface.DriveMecanum.getAxisY() != 0)
        ||
        (HumanInterface.DriveMecanum.getAxisZ() != 0)) {
      isAligning = false;
      return true;
    // If the pathfollowing command is finished stop this command
    } else if (mecanumController.isFinished()) {
      isAligning = false;
      return true;
    } else {
      isAligning = true;
      return false;
    }
  }
}
