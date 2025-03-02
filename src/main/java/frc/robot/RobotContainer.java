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
    drivCont.A.onTrue(new InstantCommand(() -> {
      elevator.retract();
      algaeArm.retract();
    }));

    // left bumper is coral/algae spinners
    opCont.A.and(opCont.RightBumper.negate()).onTrue(new InstantCommand(() -> coralManipulator.setPower(1)));
    opCont.Y.and(opCont.RightBumper.negate()).onTrue(new InstantCommand(() -> coralManipulator.setPower(-1)));
    opCont.X.and(opCont.RightBumper.negate()).onTrue(new InstantCommand(() -> coralManipulator.setPower(-0.25)));
    opCont.B.and(opCont.RightBumper.negate()).onTrue(new InstantCommand(() -> coralManipulator.setPower(0)));

    opCont.PovDown.and(opCont.RightBumper.negate()).onTrue(new InstantCommand(() -> algaeClaw.setPower(1)));
    opCont.PovUp.and(opCont.RightBumper.negate()).onTrue(new InstantCommand(() -> algaeClaw.setPower(-1)));
    opCont.PovLeft.and(opCont.RightBumper.negate()).onTrue(new InstantCommand(() -> algaeClaw.setPower(-0.25)));
    opCont.PovRight.and(opCont.RightBumper.negate()).onTrue(new InstantCommand(() -> algaeClaw.setPower(0)));

    opCont.ESTOPCondition.onTrue(new InstantCommand(Estopable::KILLIT));
    drivCont.ESTOPCondition.onTrue(new InstantCommand(Estopable::KILLIT));

    opCont.Start.onTrue(new InstantCommand(() -> algaeArm.getEncoder(0).setPosition(0)));
    opCont.Options
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

    boolean maybe = false;
    if (maybe) {
      drivCont.PovUp.and(drivCont.A)
          .whileTrue(elevator.sysID.dynamicForwards());
      drivCont.PovUp.and(drivCont.B)
          .whileTrue(elevator.sysID.dynamicReverse());

      drivCont.PovUp.and(drivCont.X)
          .whileTrue(elevator.sysID.quasistaticForwards());
      drivCont.PovUp.and(drivCont.Y)
          .whileTrue(elevator.sysID.quasistaticReverse());

      drivCont.PovDown.and(drivCont.A)
          .whileTrue(drivetrain.dynamicSysID(SysIdRoutine.Direction.kForward));
      drivCont.PovDown.and(drivCont.B)
          .whileTrue(drivetrain.dynamicSysID(SysIdRoutine.Direction.kReverse));

      drivCont.PovDown.and(drivCont.X)
          .whileTrue(drivetrain.quasistaticSysID(SysIdRoutine.Direction.kForward));
      drivCont.PovDown.and(drivCont.Y)
          .whileTrue(drivetrain.quasistaticSysID(SysIdRoutine.Direction.kReverse));
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
