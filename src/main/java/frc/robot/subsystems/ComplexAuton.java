// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Auton;
import frc.robot.Constants.PIDControllers.Holonomic;
import frc.robot.Constants.PIDControllers.WheelVelocities;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * PathPlanner implementation auton
 */
public class ComplexAuton extends SubsystemBase {
  public static SimpleMotorFeedforward feedforward;

  private static DriveTrain sysDriveTrain;

  public ComplexAuton(DriveTrain inSysDriveTrain) {
    sysDriveTrain = inSysDriveTrain;
    feedforward = new SimpleMotorFeedforward(
        Auton.FeedForwardControllers.STATIC_GAIN,
        Auton.FeedForwardControllers.VELOCITY_GAIN,
        Auton.FeedForwardControllers.ACCEL_GAIN);
  }

  /**
   * Loads the data from the files at the given paths into variables.
   * 
   * @param paths The paths to the trajectories in the form
   *              "paths/YourPath.wpilib.json"
   * @return The an array containing the loaded trajectories in the same order
   *         that they were loaded
   */
  public static Trajectory[] loadPaths(String[] paths) throws IOException {
    Trajectory[] trajectories = new Trajectory[paths.length];
    Path fs = Filesystem.getDeployDirectory().toPath();
    for (int i = 0; i != paths.length; i++) {
      try {
        Path f_path = fs.resolve(paths[i]);
        trajectories[i] = TrajectoryUtil.fromPathweaverJson(f_path);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to %open trajectory: " + paths[i], ex.getStackTrace());
        throw ex;
      }
    }
    return trajectories;
  }

  /**
   * Creates a path-following command from the given trajectory which resets odometry, drives the trajectory, and stops the DriveTrain.
   * @param pathToFollow The trajectory to use in the mecanumControllerCommand.
   * @return The path-following Command (The MecanumControllerCommand with added odometry reset and DriveTrain stop)
   */
  public static Command createDriveCommand(Trajectory pathToFollow) {
    return new SequentialCommandGroup(
      // Reset odometry
      new InstantCommand(() -> {
        DriveTrain.Odometry.resetDriveOdometry(pathToFollow.getInitialPose());
      }),
      
      // Generate drive command
      new MecanumControllerCommand(
        pathToFollow,
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
        sysDriveTrain),

        // Stop DriveTrain
        new InstantCommand(() -> {
          DriveTrain.mecanum.driveCartesian(0, 0, 0);
        })
   );
  }

  /**
   * Gets the pose of the robot using an AprilTag if avalible, and uses the
   * avalible odometry if there is not.
   * 
   * @return The position of the bot as a <b>Pose2d</b> instance.
   */
  public static Supplier<Pose2d> getPoseDual() {
    if (LimeLight.tagID != 0)
      return (Supplier<Pose2d>) () -> LimeLight.getBotPose2d();
    return (Supplier<Pose2d>) () -> DriveTrain.Odometry.mecanumDriveOdometry.getPoseMeters();
  }
}
