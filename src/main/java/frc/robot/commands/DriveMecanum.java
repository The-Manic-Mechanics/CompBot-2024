// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveMecanum extends CommandBase {
  /** Creates a new DriveMecanum. */
  private final DriveTrain sysDriveTrain;

  double speedMultThrot;
  Timer matchTimer;
  /** d = drift, ct = current time, o = old */
  double moveSpeedX, xvar, xvar_offset = 0.0, controllerd_stx = 0.0d, controllerd_oxvar;
  double moveSpeedY, yvar, yvar_offset = 0.0, controllerd_sty = 0.0d, controllerd_oyvar;
  double moveSpeedZ, zvar, zvar_offset = 0.0, controllerd_stz = 0.0d, controllerd_ozvar;


  public DriveMecanum(DriveTrain inSysDriveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysDriveTrain = inSysDriveTrain;

    addRequirements(sysDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    // if (RobotContainer.driverMainController.getRightTriggerAxis() != 0) {
    //   moveSpeedX = RobotContainer.driverMainController.getRightTriggerAxis();
    // }
    xvar = RobotContainer.driverMainController.getX();
    yvar = RobotContainer.driverMainController.getY();
    zvar = RobotContainer.driverMainController.getZ();

    if (RobotContainer.driverMainController.getThrottle() == 0) {
      speedMultThrot = RobotContainer.driverMainController.getThrottle() + .01;
    } else {
      speedMultThrot = RobotContainer.driverMainController.getThrottle();
    }

    // if (Math.abs(RobotContainer.driverMainController.getY()) > .045) {
    //   moveSpeedY = -speedMultThrot * RobotContainer.driverMainController.getY();
    // } else {
    //   moveSpeedY = 0;
    // }
    
    // Y_THROTTLE
    {
      if (controllerd_sty == 0) {
        if (yvar < .9) {
          controllerd_oyvar = yvar;
          controllerd_sty = matchTimer.getFPGATimestamp();
        }
        moveSpeedY = speedMultThrot * yvar;
      } else {
        if (Math.abs(yvar - controllerd_oyvar) >= 0.2) {
          yvar_offset = 0;
          controllerd_sty = 0;
          moveSpeedY = speedMultThrot * yvar;
          return;
        }
        if (Math.abs(controllerd_sty - matchTimer.getFPGATimestamp()) >= .5d && Math.abs(yvar - controllerd_oyvar) <= 0.2) {
          yvar_offset = yvar;
        }
      }
    }
    
    // if (Math.abs(RobotContainer.driverMainController.getX()) > .045) {
    //   moveSpeedX = -speedMultThrot * RobotContainer.driverMainController.getX();
    // } else {
    //   moveSpeedX = 0;
    // }

    // X_THROTTLE
    {
      if (controllerd_stx == 0) {
        if (xvar < .9) {
          controllerd_oxvar = xvar;
          controllerd_stx = matchTimer.getFPGATimestamp();
        }
        moveSpeedX = speedMultThrot * xvar;
      } else {
        if (Math.abs(xvar - controllerd_oxvar) >= 0.2) {
          xvar_offset = 0;
          controllerd_stx = 0;
          moveSpeedX = speedMultThrot * xvar;
          return;
        }
        if (Math.abs(controllerd_stx - matchTimer.getFPGATimestamp()) >= .5d && Math.abs(xvar - controllerd_oxvar) <= 0.2) {
          xvar_offset = xvar;
        }
      }
    }
    
    // if (Math.abs(RobotContainer.driverMainController.getZ()) > .062) {
    // moveSpeedZ = -speedMultThrot  * RobotContainer.driverMainController.getZ();
    // } else {
    //   moveSpeedZ = 0;
    // }
    // Z_THROTTLE
    {
      if (controllerd_stz == 0) {
        if (zvar < .9) {
          controllerd_ozvar = zvar;
          controllerd_stz = matchTimer.getFPGATimestamp();
        }
        moveSpeedZ = speedMultThrot * zvar;
      } else {
        if (Math.abs(zvar - controllerd_ozvar) >= 0.2) {
          zvar_offset = 0;
          controllerd_stz = 0;
          moveSpeedZ = speedMultThrot * zvar;
          return;
        }
        if (Math.abs(controllerd_stz - matchTimer.getFPGATimestamp()) >= .5d && Math.abs(zvar - controllerd_ozvar) <= 0.2) {
          zvar_offset = zvar;
        }
      }
    }

    sysDriveTrain.CartisianDrive(moveSpeedY - yvar_offset, moveSpeedX - xvar_offset, moveSpeedZ - zvar_offset);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysDriveTrain.CartisianDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
