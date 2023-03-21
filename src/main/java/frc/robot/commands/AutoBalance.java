// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VMXPiConstants.AutoBalanceConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VMXPi;

public class AutoBalance extends CommandBase {
  private final VMXPi sysVmxPi;
  private final DriveTrain sysDriveTrain;
  public boolean balanceOnOffset;
  /*Offset Threshold in degrees from zero*/
  public double offSetThesh;
   /** Creates a new AutoBalance. */
  public AutoBalance(VMXPi inSysVMXPi, DriveTrain inSysDriveTrain, 
  boolean inBalanceOnOffset, double inOffsetThresh
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysVmxPi = inSysVMXPi;
    sysDriveTrain = inSysDriveTrain;
    balanceOnOffset = inBalanceOnOffset;
    offSetThesh = inOffsetThresh;

    addRequirements(sysVmxPi, sysDriveTrain);
  }
  
  public boolean isBalanced;

  // Current roll of the VMX Pi 
  public double currentRoll;

  // Speed for the motors
  double speed;
  
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isBalanced = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Getting the roll from the VMX Pi
    currentRoll = sysVmxPi.GetRollVMXPI();

    speed = sysVmxPi.CalculateAutoBalancePID();

    if (speed > 1) speed = 1;

    // if: waits until the offset is passed to autobalance, 
    // else: autobalance when the command is scheduled

    if (balanceOnOffset && !isBalanced) {
      if (currentRoll > offSetThesh) {
        sysDriveTrain.CartisianDrive(speed, 0, 0);
      }
    } else {
      if (!isBalanced) {
        sysDriveTrain.CartisianDrive(speed, 0, 0);
      } 
    }

    if (sysVmxPi.AutoBalancePIDAtSetpoint()) {
      isBalanced = true;
    } else {
      isBalanced = false;
    }
  }
   

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysDriveTrain.CartisianDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double lgety = RobotContainer.driverMainController.getLeftY();
    double rgetx = RobotContainer.driverMainController.getLeftX();
    if ((AutoBalanceConstants.DEADZONE_MIN >= currentRoll) 
    ||
    (AutoBalanceConstants.DEADZONE_MAX <= currentRoll)) {
      return true;
    } else if (isBalanced) {
      return true;

    // #TODO# Move this shutoff into RobotContainer or make it faster in some other way
    // 9 corresponds to joystick labels
    } else if (RobotContainer.driverMainController.getRawButtonPressed(9) == true) {
      return true;

    } else if (
      (
      lgety > AutoBalanceConstants.DISABLETHRESHOLD_CONTROLLER_LEFTSTICK_MAX 
      ||
      lgety < AutoBalanceConstants.DISABLETHRESHOLD_CONTROLLER_LEFTSTICK_MIN
      ) || (
      rgetx > AutoBalanceConstants.DISABLETHRESHOLD_CONTROLLER_RIGHTSTICK_MAX
      ||
      rgetx < AutoBalanceConstants.DISABLETHRESHOLD_CONTROLLER_RIGHTSTICK_MIN
      )
      ) {
      return true;

    } else {
      return false;
    }
  }
}
