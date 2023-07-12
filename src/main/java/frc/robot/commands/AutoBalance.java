// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

import static frc.robot.Constants.AutoBalance.*;

/**
* Command that uses a PID loop to attempt to drive the robot to level when ran
*/
public final class AutoBalance extends CommandBase {
	public boolean balanceOnOffset;
	/**
	 * Offset Threshold in degrees from zero
	 */
	public double offSetThreshold;

    /**
     * @param inBalanceOnOffset Whether to balance immediately or to wait for an offset
     * @param inOffsetThresh The offset threshold before the program begins the balancing routine
    */
	public AutoBalance(NavX inSysNavX, DriveTrain inSysDriveTrain, boolean inBalanceOnOffset, double inOffsetThresh) {
		balanceOnOffset = inBalanceOnOffset;
		offSetThreshold = inOffsetThresh;
		addRequirements(inSysNavX, inSysDriveTrain);
	}

	public boolean isBalanced;
	
	@Override
	public void initialize() {
		isBalanced = false;
		NavX.autoBalanceCommandIsActive = true;
	}
	
	@Override
	public void execute() {
		/**
		* Speed for the motors
		*/
		double speed = NavX.CalculateAutoBalancePID();

		if (speed > 1)
			speed = 1;

		// if: waits until the offset is passed to auto-balance,
		// else: auto-balance when the command is scheduled

		if (!balanceOnOffset || NavX.sensor.getPitch() > offSetThreshold)
			DriveTrain.mecanum.driveCartesian(speed, 0, 0);
		else {
			isBalanced = true;
			speed = 0;
		}
	}

	
	@Override
	public void end(boolean interrupted) {
		DriveTrain.mecanum.driveCartesian(0, 0, 0);
	}
	
	@Override
	public boolean isFinished() {
		double currentPitch = NavX.sensor.getPitch();
		double lGetY = RobotContainer.driverMainController.getLeftY();
		double rGetX = RobotContainer.driverMainController.getLeftX();
		// Checks if the current pitch is outside the deadzone and finishes if it is
		if (
						(DEADZONE_MIN >= currentPitch)
						||
						(DEADZONE_MAX <= currentPitch)
		) {
			return true;
		/* Otherwise it checks if the robot is currently balanced and
		if it is it returns true and shows inactive on the smart dashboard */
		} else if (isBalanced) {
			NavX.autoBalanceCommandIsActive = false;
			return true;
			// FIXME: Figure this out once we can test.
			// TODO: Move this shutoff into RobotContainer or make it faster in some other way
			// 9 corresponds to joystick labels
		} else if (RobotContainer.driverMainController.getRawButtonPressed(9)) {
			return true;

		} else
			return
					(
							lGetY > DisableThresholds.LEFTSTICK_MAX
									||
							lGetY < DisableThresholds.LEFTSTICK_MIN
					)
							||
					(
							rGetX > DisableThresholds.RIGHTSTICK_MAX
									||
							rGetX < DisableThresholds.RIGHTSTICK_MIN
					);
	}
}
