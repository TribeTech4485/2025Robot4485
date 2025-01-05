// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.SyncedLibraries.SystemBases.PhotonVisionBase;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;

public class MoveToDistanceApriltag extends Command {
  /** Creates a new MoveToDistanceApriltag. */
  Drivetrain driveBase;
  PhotonVisionBase photon;
  double desiredX;
  double desiredY;
  double desiredTheta;
  boolean fieldRelative;
  PIDController xController = new PIDController(0.1, 0.0, 0.0);
  PIDController yController = new PIDController(0.1, 0.0, 0.0);
  PIDController thetaController = new PIDController(0.1, 0.0, 0.0);

  public MoveToDistanceApriltag(Drivetrain drivetrain, PhotonVisionBase photonVision, double distanceX, double distanceY, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, photonVision);
    driveBase = drivetrain;
    photon = photonVision;
    desiredX = distanceX;
    desiredY = distanceY;
    desiredTheta = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();

    xController.setSetpoint(desiredX);
    yController.setSetpoint(desiredY);
    thetaController.setSetpoint(desiredTheta);
    
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    thetaController.setTolerance(0.1);

    fieldRelative = driveBase.getFieldRelative();
    driveBase.setFieldRelative(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonTrackedTarget target = photon.mainTarget;
    Transform3d transform = target.bestCameraToTarget;
    Translation3d translation = transform.getTranslation();
    double x = translation.getX();
    double y = translation.getY();
    double theta = transform.getRotation().getAngle();
    
    double xOutput = xController.calculate(x);
    double yOutput = yController.calculate(y);
    double thetaOutput = thetaController.calculate(theta);

    driveBase.inputDrivingX_Y(xOutput, yOutput, thetaOutput, -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.stop();
    driveBase.setFieldRelative(fieldRelative);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}
