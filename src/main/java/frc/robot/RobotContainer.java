// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.MoveToDistanceApriltag;
import frc.robot.Commands.TeleDrive;
import frc.robot.Subsystems.CoralManipulator;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.PhotonVision;
import frc.robot.Subsystems.AlgaeArm;
import frc.robot.Subsystems.AlgaeClaw;
import frc.robot.Subsystems.Controllers2025;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Subsystems.Swerve.HoloDrive;
import frc.robot.SyncedLibraries.SystemBases.ControllerBase;
import frc.robot.SyncedLibraries.SystemBases.Estopable;
import frc.robot.SyncedLibraries.SystemBases.PathPlanning.HolonomicDriveBase;
import frc.robot.SyncedLibraries.SystemBases.PathPlanning.TrajectoryMoveCommand;
import frc.robot.SyncedLibraries.SystemBases.Utils.BackgroundTrajectoryGenerator;

public class RobotContainer {
  Controllers2025 controllers = new Controllers2025();
  ControllerBase drivCont = controllers.Zero;
  ControllerBase opCont = controllers.One;

  Drivetrain drivetrain = new Drivetrain();
  // PhotonVision photon = new PhotonVision();
  PowerDistribution pdh = new PowerDistribution(20, ModuleType.kRev);
  AlgaeClaw algaeClaw = new AlgaeClaw();
  Elevator elevator = new Elevator();
  AlgaeArm algaeArm = new AlgaeArm(elevator);
  CoralManipulator coralManipulator = new CoralManipulator();
  TeleDrive teleDrive = new TeleDrive(drivetrain, controllers, elevator, algaeArm, algaeClaw, coralManipulator);
  HolonomicDriveBase holoDrive = new HoloDrive(drivetrain);
  BackgroundTrajectoryGenerator generator = new BackgroundTrajectoryGenerator(
      new Pose2d(),
      new Pose2d(),
      List.of(
          new Translation2d(Feet.of(0), Feet.of(3)),
          new Translation2d(Feet.of(3), Feet.of(0)),
          new Translation2d(Feet.of(-2), Feet.of(0))),
      drivetrain, 0.15);

  // MoveToDistanceApriltag moveToDistanceApriltag = new
  // MoveToDistanceApriltag(drivetrain, photon, 1, 0, 0);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
  }

  private void configureBindings() {
    // CommandScheduler.getInstance().getActiveButtonLoop().clear();
    // driverController.A.onTrue(moveToDistanceApriltag);
    teleDrive.setNormalTriggerBinds();

    if (drivCont.isXbox) {
      drivCont.A.onTrue(new InstantCommand(() -> {
        elevator.retract();
        algaeArm.retract();
      }));
    } else {
      // drivCont.buttons[6].onTrue(new InstantCommand(() -> {
      // elevator.retract();
      // algaeArm.retract();
      // }));
    }

    drivCont.ESTOPCondition.onTrue(new InstantCommand(Estopable::KILLIT));
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

      opCont.ESTOPCondition.onTrue(new InstantCommand(Estopable::KILLIT));
      drivCont.ESTOPCondition.onTrue(new InstantCommand(Estopable::KILLIT));

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
      // algae mode, no trigger
      opCont.PovUp.and(opCont.RightBumper.negate())
          .onTrue(new InstantCommand(elevator::positionTop))
          .onTrue(new InstantCommand(() -> algaeArm.moveToPosition(45)));
      opCont.PovRight.and(opCont.RightBumper.negate())
          .onTrue(new InstantCommand(elevator::positionAlgaeHigh))
          .onTrue(new InstantCommand(algaeArm::positionOut));
      opCont.PovLeft.and(opCont.RightBumper.negate())
          .onTrue(new InstantCommand(elevator::positionAlgaeLow))
          .onTrue(new InstantCommand(algaeArm::positionOut));
      opCont.PovDown.and(opCont.RightBumper.negate())
          .onTrue(new InstantCommand(elevator::positionAlgaeGround))
          .onTrue(new InstantCommand(algaeArm::positionGroundIntake));
      opCont.LeftBumper.and(opCont.RightBumper.negate())
          .onTrue(new InstantCommand(() -> elevator.moveToPosition(Inches.of(21))))
          .onTrue(new InstantCommand(algaeArm::positionOut));

      opCont.A.and(opCont.RightBumper.negate())
          .onTrue(new InstantCommand(algaeClaw::outtake));
      opCont.Y.and(opCont.RightBumper.negate())
          .onTrue(new InstantCommand(algaeClaw::intake));
      opCont.B.and(opCont.RightBumper.negate())
          .onTrue(new InstantCommand(() -> algaeClaw.setPower(0)));

      // coral mode, with trigger
      opCont.PovUp.and(opCont.RightBumper)
          .onTrue(new InstantCommand(elevator::positionTop))
          .onTrue(new InstantCommand(algaeArm::retract));
      opCont.PovRight.and(opCont.RightBumper)
          .onTrue(new InstantCommand(elevator::positionL3))
          .onTrue(new InstantCommand(algaeArm::retract));
      opCont.PovLeft.and(opCont.RightBumper)
          .onTrue(new InstantCommand(elevator::positionL2))
          .onTrue(new InstantCommand(algaeArm::retract));
      opCont.PovDown.and(opCont.RightBumper)
          .onTrue(new InstantCommand(elevator::positionL1))
          .onTrue(new InstantCommand(algaeArm::retract));
      opCont.Y.and(opCont.RightBumper)
          .onTrue(new InstantCommand(coralManipulator::outtake));
      opCont.A.and(opCont.RightBumper)
          .onTrue(new InstantCommand(coralManipulator::intake));
      opCont.B.and(opCont.RightBumper)
          .onTrue(new InstantCommand(coralManipulator::stop));

      opCont.LeftStickPress
          .onTrue(new SequentialCommandGroup(
              new WaitCommand(0.1),
              new InstantCommand(elevator::retract)))
          .onTrue(new InstantCommand(algaeArm::retract));

      if (drivCont.isJoystick) {
        drivCont.buttons[12]
            .onTrue(new InstantCommand(elevator::retract))
            .onTrue(new InstantCommand(algaeArm::retract));

        drivCont.buttons[10] // Right one
            .onTrue(new InstantCommand(() -> elevator._setPosition(Constants.Elevator.positionBoundsMin)));

        drivCont.buttons[11]
            .whileTrue(new TrajectoryMoveCommand(generator, holoDrive, true));
      } else {
        drivCont.A
            .onTrue(new InstantCommand(elevator::retract))
            .onTrue(new InstantCommand(algaeArm::retract));
        drivCont.B
            .onTrue(new InstantCommand(algaeArm::retract));
        drivCont.X
            .whileTrue(new TrajectoryMoveCommand(generator, holoDrive, true));
      }
      opCont.Options.onTrue(new InstantCommand(() -> algaeArm.getEncoder(0).setPosition(0)));
      opCont.Start.onTrue(new InstantCommand(() -> elevator._setPosition(Constants.Elevator.positionBoundsMin)));
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
