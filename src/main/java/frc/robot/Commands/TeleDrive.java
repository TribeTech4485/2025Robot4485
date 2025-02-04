package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.AlgaeArm;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.AlgaeClaw;
import frc.robot.Subsystems.CoralManipulator;
import frc.robot.SyncedLibraries.Controllers;
import frc.robot.SyncedLibraries.SystemBases.TeleDriveCommandBase;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;

public class TeleDrive extends TeleDriveCommandBase {
  final Elevator elevator;
  final AlgaeArm algaeArm;
  final AlgaeClaw algaeClaw;
  final CoralManipulator coralManipulator;

  final double elevatorPositionMiddle;
  final double elevatorPositionRadius;

  final double armPositionMiddle;
  final double armPositionRadius;

  // Percentage from center to edge, once outside:
  // the power goes to reverse in opposite the direction
  final double powerControlMaxSafeMove = 0.75;
  final double powerControlReversePower = 0.5;

  public TeleDrive(SwerveDriveBase driveTrain, Controllers controllers, Elevator elevator, AlgaeArm algaeArm,
      AlgaeClaw algaeClaw, CoralManipulator coralManipulator) {
    super(driveTrain, controllers.Zero, controllers.One);
    this.elevator = elevator;
    this.algaeArm = algaeArm;
    this.algaeClaw = algaeClaw;
    this.coralManipulator = coralManipulator;

    elevatorPositionMiddle = (Constants.Elevator.positionBoundsMax + Constants.Elevator.positionBoundsMin) / 2;
    elevatorPositionRadius = elevatorPositionMiddle - Constants.Elevator.positionBoundsMin;
    armPositionMiddle = (Constants.AlgaeArm.positionBoundsMax + Constants.AlgaeArm.positionBoundsMin) / 2;
    armPositionRadius = armPositionMiddle - Constants.AlgaeArm.positionBoundsMin;

    this.controllers[1].RightTrigger
        .onFalse(new InstantCommand(() -> elevator.moveToPosition(elevator.getPosition())))
        .onFalse(new InstantCommand(() -> algaeArm.moveToPosition(algaeArm.getPosition())));

    this.controllers[1].LeftTrigger
        .onFalse(new InstantCommand(() -> elevator.moveToPosition(elevator.getPosition())))
        .onFalse(new InstantCommand(() -> algaeArm.moveToPosition(algaeArm.getPosition())));
  }

  @Override
  public void execute() {
    super.execute();

    // Right trigger = move to position
    if (controllers[1].RightTrigger.getAsBoolean()) {
      if (controllers[1].RightJoyMoved.getAsBoolean()) {
        elevator.moveToPosition(elevatorPositionMiddle + elevatorPositionRadius * controllers[1].getRightY());
      } else {
        elevator.moveToPosition(elevator.getPosition());
      }

      if (controllers[1].LeftJoyMoved.getAsBoolean()) {
        Rotation2d rotation = new Rotation2d(controllers[1].getLeftX(), controllers[1].getLeftY());
        if (rotation.getDegrees() > 90 || rotation.getDegrees() < -90) {
          rotation = rotation.unaryMinus();
        }
        algaeArm.moveToPosition(rotation.getRadians());
      } else {
        algaeArm.moveToPosition(algaeArm.getPosition());
      }
    }

    // Left trigger = power control
    if (controllers[1].LeftTrigger.getAsBoolean()) {
      if (controllers[1].getRightY() > 0) {
        if (elevator.getPosition() > elevatorPositionMiddle
            + elevatorPositionRadius * powerControlMaxSafeMove) {
          elevator.setPower(controllers[1].getRightY());
        } else {
          elevator.setPower(-powerControlReversePower);
        }
      } else if (controllers[1].getRightY() < 0) {
        if (elevator.getPosition() < elevatorPositionMiddle
            - elevatorPositionRadius * powerControlMaxSafeMove) {
          elevator.setPower(controllers[1].getRightY());
        } else {
          elevator.setPower(powerControlReversePower);
        }
      } else {
        elevator.setPower(0);
      }

      if (controllers[1].getLeftY() > 0) {
        if (algaeArm.getPosition() > armPositionMiddle + armPositionRadius * powerControlMaxSafeMove) {
          algaeArm.setPower(controllers[1].getLeftY());
        } else {
          algaeArm.setPower(-powerControlReversePower);
        }
      } else if (controllers[1].getLeftY() < 0) {
        if (algaeArm.getPosition() < armPositionMiddle - armPositionRadius * powerControlMaxSafeMove) {
          algaeArm.setPower(controllers[1].getLeftY());
        } else {
          algaeArm.setPower(powerControlReversePower);
        }
      } else {
        algaeArm.setPower(0);
      }
    }
  }
}
