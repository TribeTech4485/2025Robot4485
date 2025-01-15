// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveModuleBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SwerveDriveBase {
	public Drivetrain() {
		super(Units.inchesToMeters(Constants.Swerve.sideLength),
				Units.inchesToMeters(Constants.Swerve.sideLength),
				new SwerveModule[] {
						new SwerveModule(
								new CANSparkMax(Constants.Wirings.swerveModule2DriveMotor, MotorType.kBrushless),
								new CANSparkMax(Constants.Wirings.swerveModule2TurningMotor, MotorType.kBrushless),
								Constants.Swerve.module2Offset, Constants.Swerve.module2Name), // front left, 3, 4

						new SwerveModule(
								new CANSparkMax(Constants.Wirings.swerveModule1DriveMotor, MotorType.kBrushless),
								new CANSparkMax(Constants.Wirings.swerveModule1TurningMotor, MotorType.kBrushless),
								Constants.Swerve.module1Offset, Constants.Swerve.module1Name), // front right 1, 2

						new SwerveModule(
								new CANSparkMax(Constants.Wirings.swerveModule3DriveMotor, MotorType.kBrushless),
								new CANSparkMax(Constants.Wirings.swerveModule3TurningMotor, MotorType.kBrushless),
								Constants.Swerve.module3Offset, Constants.Swerve.module3Name), // back left 5, 6

						new SwerveModule(
								new CANSparkMax(Constants.Wirings.swerveModule4DriveMotor, MotorType.kBrushless),
								new CANSparkMax(Constants.Wirings.swerveModule4TurningMotor, MotorType.kBrushless),
								Constants.Swerve.module4Offset, Constants.Swerve.module4Name) // back right 7, 8
				// drive motors are odd, turning motors are even
				}, Constants.Swerve.drivePID,
				Constants.Swerve.turningPID,
				Constants.Swerve.botTurnPID,
				Constants.Swerve.maxWheelSpeed, 
				Constants.Swerve.maxRotationSpeed,
				Constants.Swerve.driveAmps,
				Constants.Swerve.turnAmps);

		for (SwerveModuleBase module : modules) {
			module.inputDriveTrain(this);
		}
	}
}