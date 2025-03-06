package frc.robot.Commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  final Distance elevatorPositionMiddle;
  final Distance elevatorPositionRadius;

  final Angle armPositionMiddle;
  final Angle armPositionRadius;

  // Percentage from center to edge, once outside:
  // the power goes to reverse in opposite the direction
  final double powerControlMaxSafeMoveArm = 0.75;
  final double powerControlMaxSafeMoveEle = 0.75;
  final double powerControlReversePowerArm = 0.5;
  final double powerControlReversePowerEle = 0.5;

  public TeleDrive(SwerveDriveBase driveTrain, Controllers controllers, Elevator elevator, AlgaeArm algaeArm,
      AlgaeClaw algaeClaw, CoralManipulator coralManipulator) {
    super(driveTrain, controllers.Zero, controllers.One);
    this.elevator = elevator;
    this.algaeArm = algaeArm;
    this.algaeClaw = algaeClaw;
    this.coralManipulator = coralManipulator;

    elevatorPositionMiddle = (Constants.Elevator.positionBoundsMax.plus(Constants.Elevator.positionBoundsMin)).div(2);
    elevatorPositionRadius = elevatorPositionMiddle.minus(Constants.Elevator.positionBoundsMin);
    armPositionMiddle = (Constants.AlgaeArm.positionBoundsMax.plus(Constants.AlgaeArm.positionBoundsMin)).div(2);
    armPositionRadius = armPositionMiddle.minus(Constants.AlgaeArm.positionBoundsMin);

    this.controllers[1].RightTrigger
        .onFalse(new InstantCommand(() -> elevator.moveToPosition(elevator.getPosition())))
        .onFalse(new InstantCommand(() -> algaeArm.moveToPosition(algaeArm.getAngle())));

    this.controllers[1].LeftTrigger
        .onTrue(new InstantCommand(() -> elevator.stopCommand()))
        .onTrue(new InstantCommand(() -> algaeArm.stopCommand()))
        .onFalse(new InstantCommand(() -> elevator.moveToPosition(elevator.getPosition())))
        .onFalse(new InstantCommand(() -> algaeArm.moveToPosition(algaeArm.getAngle())));
  }

  @Override
  public void execute() {
    oldExecute();
  }

  private void newExecute() {
    super.execute();

    // Right trigger = move to position
    if (controllers[1].RightTrigger.getAsBoolean()) {
      if (controllers[1].RightJoyMoved.getAsBoolean()) {
        elevator.moveToPosition(elevatorPositionMiddle.plus(elevatorPositionRadius.times(-controllers[1].getRightY())));
        SmartDashboard.putNumber("AAAAA",
            elevatorPositionMiddle.plus(elevatorPositionRadius.times(-controllers[1].getRightY())).in(Meters));
      } else {
        elevator.moveToPosition(elevator.getPosition());
      }

      if (controllers[1].LeftJoyMoved.getAsBoolean()) {
        Angle rotation = Radians.of(-Math.atan2(controllers[1].getLeftY(), Math.abs(controllers[1].getLeftX())));
        // if (rotation.getDegrees() > 90 || rotation.getDegrees() < -90) {
        // rotation = rotation.unaryMinus();
        // }
        algaeArm.moveToPosition(rotation);
      } else {
        algaeArm.moveToPosition(algaeArm.getAngle());
      }
    }

    // Left trigger = power control
    if (controllers[1].LeftTrigger.getAsBoolean()) {
      if (controllers[1].getRightY() > 0) {
        elevator.setPower(-controllers[1].getRightY(), false);
        if (elevator.getPosition()
            .compareTo(elevatorPositionMiddle.plus(elevatorPositionRadius.times(powerControlMaxSafeMoveEle))) > 0) {
        } else {
          // elevator.setPower(-powerControlReversePowerEle);
        }
      } else if (controllers[1].getRightY() < 0) {
        elevator.setPower(-controllers[1].getRightY(), false);
        if (elevator.getPosition()
            .compareTo(elevatorPositionMiddle.minus(elevatorPositionRadius.times(powerControlMaxSafeMoveEle))) < 0) {
        } else {
          // elevator.setPower(powerControlReversePowerEle);
        }
      } else {
        elevator.setPower(0);
      }

      if (controllers[1].getLeftY() > 0) {
        // if (algaeArm.getAngle()
        // .compareTo(armPositionMiddle.plus(armPositionRadius.times(powerControlMaxSafeMoveArm)))
        // > 0) {
        algaeArm.setPower(-controllers[1].getLeftY(), false);
        // } else {
        // algaeArm.setPower(-powerControlReversePowerArm);
        // }
      } else if (controllers[1].getLeftY() < 0) {
        // if (algaeArm.getAngle()
        // .compareTo(armPositionMiddle.minus(armPositionRadius.times(powerControlMaxSafeMoveArm)))
        // < 0) {
        algaeArm.setPower(-controllers[1].getLeftY(), false);
        // } else {
        // algaeArm.setPower(powerControlReversePowerArm);
        // }
      } else {
        algaeArm.setPower(0, false);
      }
    }
  }

  // the broken one
  private void oldExecute() {
    super.execute();

    if (true) {
      // Right trigger = move to position
      if (controllers[1].RightTrigger.getAsBoolean()) {
        if (controllers[1].RightJoyMoved.getAsBoolean()) {
          elevator.moveToPosition(
              elevatorPositionMiddle.plus(elevatorPositionRadius.times(-controllers[1].getRightY())));
        } else {
          elevator.moveToPosition(elevator.getPosition());
        }

        if (controllers[1].LeftJoyMoved.getAsBoolean()) {
          Angle rotation = Radians.of(-Math.atan2(controllers[1].getLeftY(), Math.abs(controllers[1].getLeftX())));
          algaeArm.moveToPosition(rotation);
        } else {
          algaeArm.moveToPosition(algaeArm.getAngle());
        }
      }

      // Left trigger = power control
      if (controllers[1].LeftTrigger.getAsBoolean()) {
        elevator.setPower(-controllers[1].getRightY(), false);

        algaeArm.setPower(-controllers[1].getLeftY(), false);
      }
    } else {
      // Right stick pressed = move to position
      if (controllers[1].RightStickPress.getAsBoolean()) {
        if (controllers[1].RightJoyMoved.getAsBoolean()) {
          elevator.moveToPosition(
              elevatorPositionMiddle.plus(elevatorPositionRadius.times(-controllers[1].getRightY())));
        } else {
          elevator.moveToPosition(elevator.getPosition());
        }

        if (controllers[1].LeftJoyMoved.getAsBoolean()) {
          Angle rotation = Radians.of(-Math.atan2(controllers[1].getLeftY(), Math.abs(controllers[1].getLeftX())));
          algaeArm.moveToPosition(rotation);
        } else {
          algaeArm.moveToPosition(algaeArm.getAngle());
        }
      }

      // Left stick pressed = power control
      if (controllers[1].LeftStickPress.getAsBoolean()) {
        elevator.setPower(-controllers[1].getRightY(), false);

        algaeArm.setPower(-controllers[1].getLeftY(), false);
      }

    }
  }
}
