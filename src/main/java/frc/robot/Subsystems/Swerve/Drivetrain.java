// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import frc.robot.SyncedLibraries.SystemBases.SwerveDriveBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SwerveDriveBase {
	SwerveModule[] modules;
	PIDController fakeController = new PIDController(0.5, 0, 0.01);

	public Drivetrain() {
		super(3, 3, new SwerveModule[] {
				new SwerveModule(new CANSparkMax(3, MotorType.kBrushless),
						new CANSparkMax(4, MotorType.kBrushless),
						0.8288317, "Front left"), // front left, 3, 4
				new SwerveModule(new CANSparkMax(1, MotorType.kBrushless),
						new CANSparkMax(2, MotorType.kBrushless),
						0.9547673, "Front right"), // front right 1, 2
				new SwerveModule(new CANSparkMax(5, MotorType.kBrushless),
						new CANSparkMax(6, MotorType.kBrushless),
						0.7699317, "Back left"), // back left 5, 6
				new SwerveModule(new CANSparkMax(7, MotorType.kBrushless),
						new CANSparkMax(8, MotorType.kBrushless),
						0.0163250, "Back right") // back right 7, 8
				// drive motors are odd, turning motors are even
		});
	}

	public void resetDriveEncoder() {
		m_frontLeft.resetDriveEncoder();
		m_frontRight.resetDriveEncoder();
		m_backLeft.resetDriveEncoder();
		m_backRight.resetDriveEncoder();
	}

	@Override
	public void periodic() {
		super.periodic();
		if (m_frontLeft.m_drivePIDController.getP() != fakeController.getP()) {
			m_frontLeft.m_drivePIDController.setP(fakeController.getP());
			m_frontRight.m_drivePIDController.setP(fakeController.getP());
			m_backLeft.m_drivePIDController.setP(fakeController.getP());
			m_backRight.m_drivePIDController.setP(fakeController.getP());
		}
		if (m_frontLeft.m_drivePIDController.getI() != fakeController.getI()) {
			m_frontLeft.m_drivePIDController.setI(fakeController.getI());
			m_frontRight.m_drivePIDController.setI(fakeController.getI());
			m_backLeft.m_drivePIDController.setI(fakeController.getI());
			m_backRight.m_drivePIDController.setI(fakeController.getI());
		}
		if (m_frontLeft.m_drivePIDController.getD() != fakeController.getD()) {
			m_frontLeft.m_drivePIDController.setD(fakeController.getD());
			m_frontRight.m_drivePIDController.setD(fakeController.getD());
			m_backLeft.m_drivePIDController.setD(fakeController.getD());
			m_backRight.m_drivePIDController.setD(fakeController.getD());
		}
		if (m_frontLeft.m_drivePIDController.getSetpoint() != fakeController.getSetpoint()) {
			m_frontLeft.m_drivePIDController.setSetpoint(fakeController.getSetpoint());
			m_frontRight.m_drivePIDController.setSetpoint(fakeController.getSetpoint());
			m_backLeft.m_drivePIDController.setSetpoint(fakeController.getSetpoint());
			m_backRight.m_drivePIDController.setSetpoint(fakeController.getSetpoint());
		}
		SmartDashboard.putData("Driving PIDs", fakeController);
	}
}