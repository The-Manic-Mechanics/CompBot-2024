// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Auton;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * PathPlanner implementation auton
 */
public class ComplexAuton extends SubsystemBase {
  public static SimpleMotorFeedforward feedforward;

  public ComplexAuton() {
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
    // Instatiate the Trajectory array
    Trajectory[] trajectories = new Trajectory[paths.length];
    Path fs = Filesystem.getDeployDirectory().toPath();
    // Loop through the paths array and load each of them
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
