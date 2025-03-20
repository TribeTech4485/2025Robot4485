package frc.robot.Commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.List;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.SyncedLibraries.SystemBases.PhotonVisionBase;
import frc.robot.SyncedLibraries.SystemBases.PathPlanning.TrajectoryMoveCommand;
import frc.robot.SyncedLibraries.SystemBases.Utils.BackgroundTrajectoryGenerator;

public class MoveToDistanceApriltag extends Command {
  /** Creates a new MoveToDistanceApriltag. */
  Drivetrain driveBase;
  PhotonVisionBase photon;
  double desiredX;
  double desiredY;
  double desiredTheta;
  boolean fieldRelative;
  TrajectoryMoveCommand moveCommand;
  HolonomicDriveController holoDrive;
  final double distanceMultilier = 1.0;

  public MoveToDistanceApriltag(Drivetrain drivetrain, HolonomicDriveController holoController,
      PhotonVisionBase photonVision, double distanceX,
      double distanceY, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, photonVision);
    driveBase = drivetrain;
    photon = photonVision;
    desiredX = distanceX;
    desiredY = distanceY;
    desiredTheta = angle;
    holoDrive = holoController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Translation3d target = photon.mainTarget.bestCameraToTarget.getTranslation();
    Translation2d targetTranslation = new Translation2d(target.getX() * distanceMultilier,
        target.getY() * distanceMultilier);
    BackgroundTrajectoryGenerator generator = new BackgroundTrajectoryGenerator(
        new Pose2d(), new Pose2d(
            targetTranslation, new Rotation2d()),
        List.of(
            new Translation2d(0.1, 0.1)),
        MetersPerSecond.of(1), MetersPerSecondPerSecond.of(5));
    moveCommand = new TrajectoryMoveCommand(generator, holoDrive, driveBase, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    moveCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    moveCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (moveCommand == null) {
      return false;
    }
    return moveCommand.isFinished();
  }
}
