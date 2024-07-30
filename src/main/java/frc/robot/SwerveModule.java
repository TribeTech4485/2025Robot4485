// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final SparkPIDController m_drivePIDController;
  private final SparkPIDController m_turningPIDController;
  int counter = 0;

  public SwerveModule(
      CANSparkMax driveMotor, CANSparkMax turningMotor) {

    // DRIVE MOTOR SETUP
    m_driveMotor = driveMotor;

    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_driveMotor.setSmartCurrentLimit(10); // TODO: increase drive current

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(1 / (Math.PI * 3 / 2 * 10 * 100));

    m_drivePIDController = m_driveMotor.getPIDController();
    m_drivePIDController.setP(0.0005);
    m_drivePIDController.setI(0.000001);
    m_drivePIDController.setD(0.02);
    m_drivePIDController.setFF(0.0002);
    m_drivePIDController.setIZone(100);

    // TURNING MOTOR SETUP
    m_turningMotor = turningMotor;

    m_turningMotor.restoreFactoryDefaults();
    m_turningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_turningMotor.setSmartCurrentLimit(10); // TODO: increase turn current

    m_turningEncoder = m_turningMotor.getEncoder();
    m_turningEncoder.setPositionConversionFactor(1 / (27 / 5 / Math.PI)); // 27:5 gear ratio

    m_turningPIDController = m_turningMotor.getPIDController();
    m_turningPIDController.setP(0.2);
    m_turningPIDController.setI(0.005);
    m_turningPIDController.setD(0.05);
    m_turningPIDController.setFF(0);
    m_turningPIDController.setIZone(0.25); // Only use I term when error is within 0.25 radians
    m_turningPIDController.setPositionPIDWrappingEnabled(true); // Wraps error around for circular inputs
    m_turningPIDController.setPositionPIDWrappingMinInput(-Math.PI);
    m_turningPIDController.setPositionPIDWrappingMaxInput(Math.PI);

    m_turningMotor.burnFlash();
    m_driveMotor.burnFlash();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // return new SwerveModuleState(
    // m_driveEncoder.getVelocity(), new
    // Rotation2d(m_turningEncoder.getPosition()));
    return new SwerveModuleState(
        m_driveMotor.get(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  public CANSparkMax getTurnMotor() {
    return m_turningMotor;
  }

  public CANSparkMax getDriveMotor() {
    return m_driveMotor;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d encoderRotation = new Rotation2d(m_turningEncoder.getPosition());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);
    // SwerveModuleState state = desiredState;

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired direction of travel that can occur when modules change
    // directions. This results in smoother driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    // m_drivePIDController.setReference(state.speedMetersPerSecond,
    // ControlType.kVelocity);
    m_driveMotor.set(state.speedMetersPerSecond);

    m_turningPIDController.setReference(state.angle.getRadians(), ControlType.kPosition);
  }

  @Override
  public void periodic() {
    if (counter++ % 10 == -1) {
      double p = SmartDashboard.getNumber("TurnP", 0);
      double i = SmartDashboard.getNumber("TurnI", 0);
      double d = SmartDashboard.getNumber("TurnD", 0);

      if (p != m_turningPIDController.getP()) {
        m_turningPIDController.setP(p);
      }
      if (i != m_turningPIDController.getI()) {
        m_turningPIDController.setI(i);
      }
      if (d != m_turningPIDController.getD()) {
        m_turningPIDController.setD(d);
      }
    }
  }
}