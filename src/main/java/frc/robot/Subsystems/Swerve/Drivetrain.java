// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveModuleBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SwerveDriveBase {
	public Drivetrain() {
		super(3.0, 3.0, new SwerveModule[] {
				new SwerveModule(new CANSparkMax(3, MotorType.kBrushless),
						new CANSparkMax(4, MotorType.kBrushless),
						0.3288317, "Front left"), // front left, 3, 4
				new SwerveModule(new CANSparkMax(1, MotorType.kBrushless),
						new CANSparkMax(2, MotorType.kBrushless),
						0.4547673, "Front right"), // front right 1, 2
				new SwerveModule(new CANSparkMax(5, MotorType.kBrushless),
						new CANSparkMax(6, MotorType.kBrushless),
						0.2699317, "Back left"), // back left 5, 6
				new SwerveModule(new CANSparkMax(7, MotorType.kBrushless),
						new CANSparkMax(8, MotorType.kBrushless),
						0.5163250, "Back right") // back right 7, 8
				// drive motors are odd, turning motors are even
		}, Constants.Swerve.drivePID, Constants.Swerve.turningPID,
				Constants.Swerve.botTurnPID, Constants.Swerve.driveAmps,
				Constants.Swerve.turnAmps);

		for (SwerveModuleBase module : modules) {
			module.inputDriveTrain(this);
		}
	}
}