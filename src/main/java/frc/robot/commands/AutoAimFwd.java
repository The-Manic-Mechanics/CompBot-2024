// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AprilTagCoordinates;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.VMXPi;

public class AutoAimFwd extends CommandBase {
  /** Creates a new AutoAim. */
  private final DriveTrain sysDriveTrain;
  private final LimeLight sysLimeLight;
  private final VMXPi sysVMXPi;

  public AutoAimFwd(DriveTrain inSysDriveTrain, LimeLight inSysLimeLight, VMXPi inSysVMXPi) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysDriveTrain = inSysDriveTrain;
    sysLimeLight = inSysLimeLight;
    sysVMXPi = inSysVMXPi;
    addRequirements(sysDriveTrain, sysLimeLight, sysVMXPi);
  }

  boolean isAimed;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isAimed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double [][] aprilTagCords = AprilTagCoordinates.APRILTAG_COORDS;

    // #TODO# Run Tests On AutoAiming
    // Use Tag 6 for testing (Blue Com Loading Side)
    PathPlannerTrajectory path = sysDriveTrain.genPath(
      1, 
      .97, 
      sysLimeLight.GetBotPose2d(), 
      sysVMXPi.vmxPi.getYaw(), // Should be 0 when alined in test
      sysVMXPi.vmxPi.getYaw(), // Theortically should be 0 when alined in test
      AprilTagCoordinates.AprilTagCoord_Trans2d(sysLimeLight.id, 1),
      aprilTagCords[(int) sysLimeLight.id][4], // 4 is heading in the raw cords
      aprilTagCords[(int) sysLimeLight.id][4]
    );

    sysDriveTrain.mecanumDriveOdometry.resetPosition(
      sysVMXPi.vmxPi.getRotation2d(), 
      sysDriveTrain.getCurMecWheelPos(), 
      new Pose2d(
        sysLimeLight.botPoseArray[1], 
        sysLimeLight.botPoseArray[2],
        sysVMXPi.vmxPi.getRotation2d()
      )
    );

    // Follows var "path"
    sysDriveTrain.followTrajectoryCommand(path, false);

    // CommandScheduler.getInstance().schedule(sysDriveTrain.followTrajectoryCommand(path, false));
    
    isAimed = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAimed;
  }
}