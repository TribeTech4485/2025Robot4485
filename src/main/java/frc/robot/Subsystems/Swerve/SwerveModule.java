// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final SparkAbsoluteEncoder m_turningEncoder;

  private final PIDController m_drivePIDController;
  private final PIDController m_turnPIDController;

  private final double driveGearRatio = 1 / (10 * Math.PI * 15 / 50); // 1 is the gear ratio when I find out

  /**
   * Constructs a new SwerveModule.
   *
   * @param driveMotor    The motor that drives the module.
   * @param turningMotor  The motor that turns the module.
   * @param turningOffset The offset for the turning encoder. Starting position
   * @param name          The name of the module. Ie. "FrontLeft"
   */
  public SwerveModule(CANSparkMax driveMotor, CANSparkMax turningMotor, double turningOffset, String name) {
    this.setName(name);

    // DRIVE MOTOR SETUP
    m_driveMotor = driveMotor;

    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_driveMotor.setSmartCurrentLimit(10); // will increase

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(driveGearRatio);
    m_driveEncoder.setVelocityConversionFactor(driveGearRatio);

    m_drivePIDController = new PIDController(0.1, 0, 0.01);

    // TURNING MOTOR SETUP
    m_turningMotor = turningMotor;

    m_turningMotor.restoreFactoryDefaults();
    m_turningMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_turningMotor.setSmartCurrentLimit(10);

    m_turningEncoder = m_turningMotor.getAbsoluteEncoder();
    m_turningEncoder.setPositionConversionFactor(1);
    m_turningEncoder.setZeroOffset(turningOffset);

    m_turnPIDController = new PIDController(1.5, 0, 0.1);
    m_turnPIDController.enableContinuousInput(0, 1);

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
        m_driveMotor.get(), getEncoderPos());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), getEncoderPos());
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d encoderRotation = getEncoderPos();

    SwerveModuleState state = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    // Optimize the reference state to avoid spinning further than 90 degrees
    state = SwerveModuleState.optimize(state, getEncoderPos());

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired direction of travel that can occur when modules change
    // directions. This results in smoother driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // set the wanted position, actual moving done in periodic
    m_drivePIDController.setSetpoint(state.speedMetersPerSecond);
    m_turnPIDController.setSetpoint(covertFromRadians(state.angle.getRadians()));
  }

  @Override
  public void periodic() {
    m_turningMotor.set(m_turnPIDController.calculate(m_turningEncoder.getPosition()));

    // m_driveMotor.set(m_drivePIDController.calculate(m_driveEncoder.getVelocity())); // TODO
    m_driveMotor.set(m_drivePIDController.getSetpoint() / 5);

    SmartDashboard.putData(this.getName() + " swerve turning PID", m_turnPIDController);
    SmartDashboard.putNumber(this.getName() + " swerve turning encoder", m_turningEncoder.getPosition());
    SmartDashboard.putNumber(this.getName() + " swerve turning degrees", getEncoderPos().getDegrees());
    SmartDashboard.putNumber(this.getName() + " swerve turning power", m_turningMotor.get());

    SmartDashboard.putData(this.getName() + " swerve driving PID", m_drivePIDController);
    SmartDashboard.putNumber(this.getName() + " swerve driving speed", m_driveEncoder.getVelocity());
    SmartDashboard.putNumber(this.getName() + " swerve driving power", m_driveMotor.get());
  }

  public void setDriveBrakeMode(boolean brake) {
    if (brake) {
      m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    } else {
      m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
  }

  private double covertFromRadians(double radians) {
    return (radians / (-2 * Math.PI)) + (1 / 2);
  }

  private double convertToRadians(double position) {
    return (-(position - (1 / 2))) * (2 * Math.PI);
  }

  /** Converts to radians */
  private Rotation2d getEncoderPos() {
    return new Rotation2d(convertToRadians(m_turningEncoder.getPosition()));
  }

  public CANSparkMax getTurnMotor() {
    return m_turningMotor;
  }

  public CANSparkMax getDriveMotor() {
    return m_driveMotor;
  }
}