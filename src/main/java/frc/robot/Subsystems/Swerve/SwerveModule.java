// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Movement;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveModuleBase;

public class SwerveModule extends SwerveModuleBase {

  private static final double[] drivePIDF = Movement.Drive.PIDF;
  private static final double[] turnPID = Movement.Turn.PID;
  private static final SparkBaseConfig driveConfig = new SparkMaxConfig()
      .smartCurrentLimit(Constants.Swerve.driveAmps)
      .inverted(true)
      .apply(new EncoderConfig().velocityConversionFactor(Movement.driveGearRatio));
  private static final SparkBaseConfig turningConfig = new SparkMaxConfig()
      .smartCurrentLimit(Constants.Swerve.turnAmps)
      // .apply(new
      // ClosedLoopConfig().p(Movement.Turn.P).i(Movement.Turn.I).d(Movement.Turn.D))
      .apply(new AbsoluteEncoderConfig().positionConversionFactor(Math.PI * 2)
          .inverted(true)
          .velocityConversionFactor(Math.PI * 2));
  private static final TrapezoidProfile.Constraints driveConstraints = Drivetrain.driveConstraints;

  public SwerveModule(SparkMax driveMotor, SparkMax turningMotor, double turningOffset, String name) {
    super(driveMotor, turningMotor, turningOffset, name,
        driveConfig, turningConfig, drivePIDF, turnPID, driveConstraints);
    breakerMaxAmps = 30;
  }
}