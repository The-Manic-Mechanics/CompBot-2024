// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.Auton;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.DriveTrain.Motors;

public class SysID extends SubsystemBase {
	SysIdRoutine routine;
  	private final MutableMeasure<Voltage> m_appliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
  	private final MutableMeasure<Distance> m_distance = MutableMeasure.mutable(Units.Meters.of(0));
  	private final MutableMeasure<Velocity<Distance>> m_velocity = MutableMeasure.mutable(Units.MetersPerSecond.of(0));
	public SysID(DriveTrain sysDriveTrain) {
		Measure<Velocity<Voltage>> rampRate = Units.Volts.of(1).per(Units.Seconds.of(5));
		Measure<Voltage> stepVoltage = Units.Volts.of(1);
		Measure<Time> timeout = Units.Second.of(10);
		routine = new SysIdRoutine(
				new SysIdRoutine.Config(rampRate, stepVoltage, timeout),
				new SysIdRoutine.Mechanism(this::driveVoltage, log -> {
                log.motor("frontLeft")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            DriveTrain.Motors.frontLeft.getBusVoltage(), Units.Volts))
                    .linearPosition(m_distance.mut_replace(DriveTrain.Encoders.frontLeft.getPosition(), Units.Inches))
                    .linearVelocity(
                        m_velocity.mut_replace((DriveTrain.Encoders.frontLeft.getVelocity() * (Auton.DISTANCE_PER_PULSE * Auton.MOTOR_COUNTS_PER_REV)) * 39.37, Units.InchesPerSecond));
                log.motor("frontRight")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            DriveTrain.Motors.frontRight.getBusVoltage(), Units.Volts))
                    .linearPosition(m_distance.mut_replace(DriveTrain.Encoders.frontRight.getPosition(), Units.Inches))

                    .linearVelocity(
                        m_velocity.mut_replace((DriveTrain.Encoders.frontRight.getVelocity() * (Auton.DISTANCE_PER_PULSE * Auton.MOTOR_COUNTS_PER_REV)) * 39.37, Units.InchesPerSecond));
				log.motor("rearLeft")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            DriveTrain.Motors.rearLeft.getBusVoltage(), Units.Volts))
                    .linearPosition(m_distance.mut_replace(DriveTrain.Encoders.rearLeft.getPosition(), Units.Inches))

                    .linearVelocity(
                        m_velocity.mut_replace(DriveTrain.Encoders.rearLeft.getVelocity() * (Auton.DISTANCE_PER_PULSE * Auton.MOTOR_COUNTS_PER_REV) * 39.37, Units.InchesPerSecond));
                log.motor("rearRight")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            DriveTrain.Motors.rearRight.getBusVoltage(), Units.Volts))
                    .linearPosition(m_distance.mut_replace(DriveTrain.Encoders.rearRight.getPosition(), Units.Inches))

                    .linearVelocity(
                        m_velocity.mut_replace(DriveTrain.Encoders.rearRight.getVelocity() * (Auton.DISTANCE_PER_PULSE * Auton.MOTOR_COUNTS_PER_REV) * 39.37, Units.InchesPerSecond));
              }, sysDriveTrain));
	}

	public Command runQuasistatic(SysIdRoutine.Direction direction) {
		return routine.quasistatic(direction);
	}

	public Command runDynamic(SysIdRoutine.Direction direction) {
		return routine.dynamic(direction);
	}

	public void driveVoltage(Measure<Voltage> driveVolts) {
		Motors.frontLeft.setVoltage(driveVolts.baseUnitMagnitude());
		Motors.frontRight.setVoltage(driveVolts.baseUnitMagnitude());
		Motors.rearLeft.setVoltage(driveVolts.baseUnitMagnitude());
		Motors.rearRight.setVoltage(driveVolts.baseUnitMagnitude());
	}
}
