// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.revrobotics.spark.SparkMax;

import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveModuleBase;

public class SwerveModule extends SwerveModuleBase {

  public SwerveModule(SparkMax driveMotor, SparkMax turningMotor, double turningOffset, String name) {
    super(driveMotor, turningMotor, turningOffset, name);
  }
}