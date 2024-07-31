// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 1.0; // 3 meters per second // TODO: 3
  public static final double kMaxAngularSpeed = 2 * Math.PI; // 1 rotation per second

  // For a square 3ft x 3ft robot, the wheelbase is 0.381 meters from center to
  // module.
  private final double _robotWidth = Units.feetToMeters(3);
  private final double _robotLength = Units.feetToMeters(3);
  private final double _robotWidthOffset = _robotWidth / 2;
  private final double _robotLengthOffset = _robotLength / 2;

  private final Translation2d m_frontLeftLocation = new Translation2d(_robotLengthOffset, _robotWidthOffset);
  private final Translation2d m_frontRightLocation = new Translation2d(_robotLengthOffset, -_robotWidthOffset);
  private final Translation2d m_backLeftLocation = new Translation2d(-_robotLengthOffset, _robotWidthOffset);
  private final Translation2d m_backRightLocation = new Translation2d(-_robotLengthOffset, -_robotWidthOffset);
  private final Translation2d m_frontLocation = new Translation2d(_robotLengthOffset, 0);
  private final Translation2d m_rightLocation = new Translation2d(0, -_robotWidthOffset);
  private final Translation2d m_backLocation = new Translation2d(-_robotLengthOffset, 0);
  private final Translation2d m_leftLocation = new Translation2d(0, _robotWidthOffset);

  private final SwerveModule m_frontLeft = new SwerveModule(
      new CANSparkMax(3, MotorType.kBrushless),
      new CANSparkMax(4, MotorType.kBrushless));
  private final SwerveModule m_frontRight = new SwerveModule(
      new CANSparkMax(1, MotorType.kBrushless),
      new CANSparkMax(2, MotorType.kBrushless));
  private final SwerveModule m_backLeft = new SwerveModule(
      new CANSparkMax(5, MotorType.kBrushless),
      new CANSparkMax(6, MotorType.kBrushless));
  private final SwerveModule m_backRight = new SwerveModule(
      new CANSparkMax(7, MotorType.kBrushless),
      new CANSparkMax(8, MotorType.kBrushless));

  int counter = 0;

  private final AHRS m_gyro = new AHRS();
  DigitalInput testSwitch = new DigitalInput(9);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      });
  private StructArrayPublisher<SwerveModuleState> NetworkTablesSwervePublisherDesired;
  private StructArrayPublisher<SwerveModuleState> NetworkTablesSwervePublisherCurrent;

  public Drivetrain() {
    m_gyro.reset();
    NetworkTablesSwervePublisherDesired = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/DesiredSwerveStates", SwerveModuleState.struct).publish();
    NetworkTablesSwervePublisherCurrent = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/CurrentSwerveStates", SwerveModuleState.struct).publish();

  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed              Speed of the robot in the x direction (forward).
   * @param ySpeed              Speed of the robot in the y direction (right).
   * @param rotationSpeed       Angular rate of the robot.
   * @param fieldRelative       Whether the provided x and y speeds are relative
   *                            to the field.
   * @param centerOfRotationPOV Input pov value where -1 is center, and 0 is front
   */
  public void drive(
      double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative, int centerOfRotationPOV) {
    xSpeed *= kMaxSpeed;
    ySpeed *= kMaxSpeed;
    rotationSpeed *= kMaxAngularSpeed;
    Translation2d centerOfRotation = POVToTranslate2d(centerOfRotationPOV);

    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        // Account for time between updates
        ChassisSpeeds.discretize(
            fieldRelative
                // if field relative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, -ySpeed, rotationSpeed,
                    m_gyro.getRotation2d())
                // if robot relative
                : new ChassisSpeeds(xSpeed, -ySpeed, rotationSpeed),
            0.02),
        centerOfRotation);

    // Constrain wheel speeds to kMaxSpeed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    counter++;
    if (counter % 10 == 0) {
      // System.out.println("Speeds: " + swerveModuleStates[0].speedMetersPerSecond +
      // " "
      // + swerveModuleStates[1].speedMetersPerSecond + " " +
      // swerveModuleStates[2].speedMetersPerSecond
      // + " " + swerveModuleStates[3].speedMetersPerSecond);
      // System.out.println("Angles: " + swerveModuleStates[0].angle.getDegrees() + "
      // "
      // + swerveModuleStates[1].angle.getDegrees() + " "
      // + swerveModuleStates[2].angle.getDegrees() + " "
      // + swerveModuleStates[3].angle.getDegrees());
    }
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    // SmartDashboard.putNumber("Speed1",
    // m_frontLeft.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("TargAngle1",
    // m_frontLeft.getState().angle.getDegrees());
    // SmartDashboard.putNumber("CurrAngle1",
    // m_frontLeft.getPosition().angle.getDegrees());
    // SmartDashboard.putNumber("Power1", m_frontLeft.getTurnMotor().get());
    SmartDashboard.putNumber("DriveSpeed", m_frontLeft.getDriveMotor().get());

    NetworkTablesSwervePublisherDesired.set(swerveModuleStates);
    NetworkTablesSwervePublisherCurrent.set(
        new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        });

    updateOdometry();
  }

  private Translation2d POVToTranslate2d(int centerOfRotation) {
    Translation2d rotationCenter;
    switch (centerOfRotation) {
      case -1:
        rotationCenter = new Translation2d(0, 0);
        break;
      case 0:
        rotationCenter = m_frontLocation;
        break;
      case 45:
        rotationCenter = m_frontRightLocation;
        break;
      case 90:
        rotationCenter = m_rightLocation;
        break;
      case 135:
        rotationCenter = m_backRightLocation;
        break;
      case 180:
        rotationCenter = m_backLocation;
        break;
      case 225:
        rotationCenter = m_backLeftLocation;
        break;
      case 270:
        rotationCenter = m_leftLocation;
        break;
      case 315:
        rotationCenter = m_frontLeftLocation;
        break;
      default:
        rotationCenter = new Translation2d(0, 0);
        break;
    }
    return rotationCenter;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
    SmartDashboard.putNumber("Gyro Rad", m_gyro.getAngle() / 180 * Math.PI);

    SmartDashboard.putBoolean("TestSwitch", !testSwitch.get());
  }
}