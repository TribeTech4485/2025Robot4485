// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveModuleBase;

public class SwerveModule extends SwerveModuleBase {

  private static final double[] drivePIDF = { 0, 0, 0, 0, 0, 0 };
  private static final double[] turnPID = { 0, 0, 0 };
  private static final SparkMaxConfig driveConfig = new SparkMaxConfig(); // TODO
  private static final SparkMaxConfig turningConfig = new SparkMaxConfig(); // TODO
  private static final TrapezoidProfile.Constraints driveConstraints = Drivetrain.driveConstraints;

  public SwerveModule(SparkMax driveMotor, SparkMax turningMotor, double turningOffset, String name) {
    super(driveMotor, turningMotor, turningOffset, name,
        driveConfig, turningConfig, drivePIDF, turnPID, driveConstraints);
    breakerMaxAmps = 30;
  }
}