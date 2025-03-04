// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.MoveToDistanceApriltag;
import frc.robot.Commands.TeleDrive;
import frc.robot.Subsystems.CoralManipulator;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.AlgaeArm;
import frc.robot.Subsystems.AlgaeClaw;
import frc.robot.Subsystems.Controllers2025;
import frc.robot.Subsystems.PhotonVision;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.SyncedLibraries.SystemBases.ControllerBase;
import frc.robot.SyncedLibraries.SystemBases.Estopable;

public class RobotContainer {
  Controllers2025 controllers = new Controllers2025();
  ControllerBase drivCont = controllers.Zero;
  ControllerBase opCont = controllers.One;
  Drivetrain drivetrain = new Drivetrain();
  // PhotonVision photon = new PhotonVision();
  AlgaeClaw algaeClaw = new AlgaeClaw();
  Elevator elevator = new Elevator();
  AlgaeArm algaeArm = new AlgaeArm(elevator);
  CoralManipulator coralManipulator = new CoralManipulator();
  TeleDrive teleDrive = new TeleDrive(drivetrain, controllers, elevator, algaeArm, algaeClaw, coralManipulator);

  // MoveToDistanceApriltag moveToDistanceApriltag = new
  // MoveToDistanceApriltag(drivetrain, photon, 1, 0, 0);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
  }

  private void configureBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    // driverController.A.onTrue(moveToDistanceApriltag);
    teleDrive.setNormalTriggerBinds();
    if (drivCont.isXbox) {
      drivCont.A.onTrue(new InstantCommand(() -> {
        elevator.retract();
        algaeArm.retract();
      }));
    } else {
      drivCont.buttons[6].onTrue(new InstantCommand(() -> {
        elevator.retract();
        algaeArm.retract();
      }));
    }

    boolean oldControls = false;
    if (oldControls) {
      // left bumper is coral/algae spinners
      opCont.A.and(opCont.RightBumper.negate()).onTrue(new InstantCommand(() -> coralManipulator.setPower(1)));
      opCont.Y.and(opCont.RightBumper.negate()).onTrue(new InstantCommand(() -> coralManipulator.setPower(-1)));
      opCont.X.and(opCont.RightBumper.negate()).onTrue(new InstantCommand(() -> coralManipulator.setPower(-0.25)));
      opCont.B.and(opCont.RightBumper.negate()).onTrue(new InstantCommand(() -> coralManipulator.setPower(0)));

      opCont.PovDown.and(opCont.RightBumper.negate()).onTrue(new InstantCommand(() -> algaeClaw.setPower(1)));
      opCont.PovUp.and(opCont.RightBumper.negate()).onTrue(new InstantCommand(() -> algaeClaw.setPower(-1)));
      opCont.PovLeft.and(opCont.RightBumper.negate()).onTrue(new InstantCommand(() -> algaeClaw.setPower(-0.25)));
      opCont.PovRight.and(opCont.RightBumper.negate()).onTrue(new InstantCommand(() -> algaeClaw.setPower(0)));

      // opCont.ESTOPCondition.onTrue(new InstantCommand(Estopable::KILLIT));
      // drivCont.ESTOPCondition.onTrue(new InstantCommand(Estopable::KILLIT));

      opCont.Start.onTrue(new InstantCommand(() -> algaeArm.getEncoder(0).setPosition(0)));
      opCont.Options // Left one
          .onTrue(new InstantCommand(() -> elevator._setPosition(Constants.Elevator.positionBoundsMin)));

      opCont.Y.and(opCont.RightBumper).onTrue(new InstantCommand(() -> {
        elevator.moveToPosition(Constants.Elevator.positionBoundsMax);
        algaeArm.moveToPosition(45);
      }));
      opCont.B.and(opCont.RightBumper).onTrue(new InstantCommand(() -> {
        elevator.moveToPosition(Feet.of(4));
        algaeArm.moveToPosition(0);
      }));
      opCont.X.and(opCont.RightBumper).onTrue(new InstantCommand(() -> {
        elevator.moveToPosition(Feet.of(2));
        algaeArm.moveToPosition(-15);
      }));
      opCont.A.and(opCont.RightBumper).onTrue(new InstantCommand(() -> {
        elevator.retract();
        algaeArm.retract();
      }));
    } else {

      Trigger bumpers = opCont.RightBumper.or(opCont.LeftBumper);
      Trigger triggers = opCont.RightTrigger.or(opCont.LeftTrigger);
      // algae mode, no trigger
      opCont.Y.and(opCont.RightBumper.negate())
          .onTrue(new InstantCommand(elevator::positionTop))
          .onTrue(new InstantCommand(algaeArm::positionOut));
      opCont.B.and(opCont.RightBumper.negate())
          .onTrue(new InstantCommand(elevator::positionAlgaeHigh))
          .onTrue(new InstantCommand(algaeArm::positionOut));
      opCont.X.and(opCont.RightBumper.negate())
          .onTrue(new InstantCommand(elevator::positionAlgaeLow))
          .onTrue(new InstantCommand(algaeArm::positionOut));
      opCont.A.and(opCont.RightBumper.negate())
          .onTrue(new InstantCommand(elevator::positionAlgaeGround))
          .onTrue(new InstantCommand(algaeArm::positionGroundIntake));
      opCont.PovLeft.and(opCont.RightBumper.negate())
          .onTrue(new InstantCommand(() -> algaeClaw.setPower(1)));
      opCont.PovRight.and(opCont.RightBumper.negate())
          .onTrue(new InstantCommand(() -> algaeClaw.setPower(-1)));
      opCont.PovDown.and(opCont.RightBumper.negate())
          .onTrue(new InstantCommand(() -> algaeClaw.setPower(0)));

      // coral mode, with trigger
      opCont.Y.and(opCont.RightBumper)
          .onTrue(new InstantCommand(elevator::positionTop))
          .onTrue(new InstantCommand(algaeArm::retract));
      opCont.B.and(opCont.RightBumper)
          .onTrue(new InstantCommand(elevator::positionL3))
          .onTrue(new InstantCommand(algaeArm::retract));
      opCont.X.and(opCont.RightBumper)
          .onTrue(new InstantCommand(elevator::positionL2))
          .onTrue(new InstantCommand(algaeArm::retract));
      opCont.A.and(opCont.RightBumper)
          .onTrue(new InstantCommand(elevator::positionL1))
          .onTrue(new InstantCommand(algaeArm::retract));
      opCont.PovLeft.and(opCont.RightBumper)
          .onTrue(new InstantCommand(() -> coralManipulator.setPower(1)));
      opCont.PovRight.and(opCont.RightBumper)
          .onTrue(new InstantCommand(() -> coralManipulator.setPower(-1)));
      opCont.PovDown.and(opCont.RightBumper)
          .onTrue(new InstantCommand(() -> coralManipulator.setPower(0)));

      drivCont.buttons[10] // Right one
          .onTrue(new InstantCommand(() -> elevator._setPosition(Constants.Elevator.positionBoundsMin)));
      opCont.Options.onTrue(new InstantCommand(() -> algaeArm.getEncoder(0).setPosition(0)));
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
