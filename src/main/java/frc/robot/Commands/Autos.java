package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems.AlgaeArm;
import frc.robot.Subsystems.AlgaeClaw;
import frc.robot.Subsystems.CoralManipulator;
import frc.robot.Subsystems.Elevator;
import frc.robot.SyncedLibraries.SystemBases.PhotonVisionBase;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;

public class Autos {
  // OLD
  public static Command coralPlace(SwerveDriveBase drivetrain, Elevator elevator,
      AlgaeArm algaeArm, CoralManipulator coralManipulator) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> algaeArm.moveToPosition(40)),
        new WaitCommand(0.5),
        new InstantCommand(elevator::positionL4),
        new InstantCommand(algaeArm::retract),
        new WaitCommand(0.5),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(
            MetersPerSecond.of(-1), MetersPerSecond.of(0), RadiansPerSecond.of(0)))
            .withTimeout(2))
        .alongWith(coralManipulator.comIntake())
        .andThen(_coralPlace(coralManipulator, elevator),
            new RunCommand(() -> drivetrain.inputDrivingX_Y(MetersPerSecond.of(0.75),
                MetersPerSecond.of(0), RadiansPerSecond.of(0))).withTimeout(2))
        .andThen(stopDrive(drivetrain))
        .andThen(new InstantCommand(algaeArm::retract),
            new InstantCommand(elevator::retract));
  }

  public static Command driveOut(SwerveDriveBase drivetrain, Elevator elevator,
      AlgaeArm algaeArm) {
    return new SequentialCommandGroup(
        new InstantCommand(algaeArm::retract),
        new InstantCommand(elevator::retract),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(
            MetersPerSecond.of(-1), MetersPerSecond.of(0), RadiansPerSecond.of(0))).withTimeout(2),
        stopDrive(drivetrain));
  }

  // OLD
  public static Command centerCoralAndAlgae(SwerveDriveBase drivetrain, Elevator elevator,
      AlgaeArm algaeArm, CoralManipulator coralManipulator) {
    return coralPlace(drivetrain, elevator, algaeArm, coralManipulator)
        .andThen(new InstantCommand());
  }

  // OLD
  private static Command _coralPlace(CoralManipulator coralManipulator, Elevator elevator) {
    return new WaitCommand(1)
        .andThen(coralManipulator.comOuttake())
        .andThen(new WaitCommand(0.5))
        .andThen(new InstantCommand(elevator::positionBarge))
        .andThen(new WaitCommand(0.5))
        .andThen(new InstantCommand(elevator::positionL4));
  }

  public static Command aprilAuto(SwerveDriveBase drivetrain, PhotonVisionBase photon, TeleDrive teleDrive,
      Elevator elevator, AlgaeArm algaeArm, CoralManipulator coralManipulator, AlgaeClaw algaeClaw) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new PrintCommand("Starting April auto"),
            coralManipulator.comIntake(),
            // new InstantCommand(teleDrive::disable),
            new InstantCommand(algaeArm::retract),
            new InstantCommand(elevator::retract),
            new InstantCommand(() -> drivetrain.setSlowMode(false)),
            centerOnAprilTag(drivetrain, photon)),
        new InstantCommand(elevator::positionL4),
        new PrintCommand("Move Elevator"),
        new WaitCommand(1),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(-0.3, 0, 0)).withTimeout(0.85),
        new ParallelCommandGroup(
            stopDrive(drivetrain),
            coralManipulator.comOuttake()),
        new InstantCommand(elevator::positionBarge),
        new WaitCommand(0.35),
        new InstantCommand(elevator::positionL4),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(0.3, 0, 0)).withTimeout(1),
        new InstantCommand(elevator::positionAlgaeLow),
        stopDrive(drivetrain),
        new PrintCommand("Finished April auto"),
        new InstantCommand(algaeArm::positionOut),
        new DumbAprilMove(drivetrain, photon, Feet.of(3), Inches.of(-10), Radians.zero())
            .withTimeout(3),
        new InstantCommand(algaeClaw::intake),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(-0.5, 0, 0)).withTimeout(0.5),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(0.5, 0, 0)).withTimeout(0.5),
        new InstantCommand(elevator::retract),
        new InstantCommand(algaeArm::retract),
        // stopDrive(drivetrain),
        // new InstantCommand(() ->
        // algaeArm.moveToPosition(Constants.AlgaeArm.positionBoundsMax.minus(Degrees.of(1)))),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(-0.4, 0, 0)).withTimeout(1),
        stopDrive(drivetrain))
        .finallyDo(() -> {
          // teleDrive.enable();
        });
  }

  /** Goes up to apriltag, hits the wall to become parallel, then backs up */
  private static Command centerOnAprilTag(SwerveDriveBase drivetrain, PhotonVisionBase photon) {
    return new SequentialCommandGroup(
        new DumbAprilMove(drivetrain, photon, Feet.of(2.5), Inches.of(-5), Radians.zero()),
        stopDrive(drivetrain).withTimeout(0.125),
        new PrintCommand("Moved infront"),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(-0.5, 0, 0)).withTimeout(1),
        new PrintCommand("Wall slammed"),
        new RunCommand(() -> drivetrain.inputDrivingX_Y(0.3, 0, 0)).withTimeout(0.5),
        stopDrive(drivetrain).withTimeout(0.5),
        new PrintCommand("Stopped"),
        new DumbAprilMove(drivetrain, photon, Feet.of(2.3), Inches.of(-5), Radians.zero())
            .withTimeout(3),
        new PrintCommand("Re-centered"))
        .finallyDo(() -> System.out.println("Finished centering?"));
  }

  private static ParallelRaceGroup stopDrive(SwerveDriveBase drivetrain) {
    return new RunCommand(drivetrain::stop).withTimeout(1);
  }
}
