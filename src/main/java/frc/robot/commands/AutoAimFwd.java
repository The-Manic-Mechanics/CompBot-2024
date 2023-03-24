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
    PathPlannerTrajectory path = sysDriveTrain.genPath(
      .97, 
      1, 
      sysLimeLight.GetBotPose2d(), 
      0, 
      sysVMXPi.vmxPi.getAngle(), 
      AprilTagCoordinates.AprilTagCoord_Trans2d(Double.parseDouble(sysLimeLight.limeLight_currentlyViewedAprilTag), 1),
      0,
      sysVMXPi.vmxPi.getAngle()
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
    CommandScheduler.getInstance().schedule(sysDriveTrain.followTrajectoryCommand(path, false));
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