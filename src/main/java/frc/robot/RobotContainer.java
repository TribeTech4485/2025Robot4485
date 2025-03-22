// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.TeleDrive;
import frc.robot.Subsystems.CoralManipulator;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.LEDS;
import frc.robot.Subsystems.PhotonVision;
import frc.robot.Subsystems.AlgaeArm;
import frc.robot.Subsystems.AlgaeClaw;
import frc.robot.Subsystems.Controllers2025;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.SyncedLibraries.SystemBases.ControllerBase;
import frc.robot.SyncedLibraries.SystemBases.Estopable;
import frc.robot.SyncedLibraries.SystemBases.Utils.BackgroundTrajectoryGenerator;

// 58 inches forward

public class RobotContainer {
  Controllers2025 controllers = new Controllers2025();
  ControllerBase drivCont = controllers.Zero;
  ControllerBase opCont = controllers.One;

  Drivetrain drivetrain = new Drivetrain();
  PhotonVision photon = new PhotonVision();
  PowerDistribution pdh = new PowerDistribution(20, ModuleType.kRev);
  AlgaeClaw algaeClaw = new AlgaeClaw();
  public Elevator elevator = new Elevator();
  AlgaeArm algaeArm = new AlgaeArm(elevator);
  CoralManipulator coralManipulator = new CoralManipulator();
  TeleDrive teleDrive = new TeleDrive(drivetrain, controllers, elevator, algaeArm, algaeClaw, coralManipulator);
  HolonomicDriveController holoDrive = new HolonomicDriveController(
      new PIDController(Constants.Swerve.Movement.Holonomic.xP,
          Constants.Swerve.Movement.Holonomic.xI, Constants.Swerve.Movement.Holonomic.xD),
      new PIDController(Constants.Swerve.Movement.Holonomic.yP,
          Constants.Swerve.Movement.Holonomic.yI, Constants.Swerve.Movement.Holonomic.yD),
      drivetrain.getTurnController());
  int ss = 2;
  BackgroundTrajectoryGenerator generator = new BackgroundTrajectoryGenerator(
      new Pose2d(),
      new Pose2d(Inches.of(140), Inches.of(0), Rotation2d.fromDegrees(0)),
      List.of(new Translation2d(0.1, 0)),
      MetersPerSecond.of(0.75), MetersPerSecondPerSecond.of(5));

  BackgroundTrajectoryGenerator generator2 = new BackgroundTrajectoryGenerator(
      new Pose2d(),
      new Pose2d(Inches.of(-40), Inches.of(0), Rotation2d.fromDegrees(0)),
      List.of(new Translation2d(-20, 0)),
      MetersPerSecond.of(1), MetersPerSecondPerSecond.of(5));
  LEDS leds = new LEDS(elevator);
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  // MoveToDistanceApriltag moveToDistanceApriltag = new
  // MoveToDistanceApriltag(drivetrain, photon, 1, 0, 0);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();

    autoChooser.setDefaultOption("Coral Place",
        new SequentialCommandGroup(
            new InstantCommand(() -> algaeArm.moveToPosition(40)),
            new WaitCommand(0.5),
            new InstantCommand(elevator::positionL4),
            new InstantCommand(algaeArm::retract),
            new WaitCommand(0.5),
            new RunCommand(() -> drivetrain.inputDrivingX_Y(
                MetersPerSecond.of(-1), MetersPerSecond.of(0), RadiansPerSecond.of(0)))
                .withTimeout(2))
            .alongWith(coralManipulator.comIntake())
            .andThen(new WaitCommand(1))
            .andThen(coralManipulator.comOuttake())
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(elevator::positionBarge))
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(elevator::positionL4),
                new RunCommand(() -> drivetrain.inputDrivingX_Y(
                    MetersPerSecond.of(0.75), MetersPerSecond.of(0), RadiansPerSecond.of(0)))
                    .withTimeout(2))
            .andThen(new RunCommand(drivetrain::stop).withTimeout(1))
            .andThen(new InstantCommand(algaeArm::retract),
                new InstantCommand(elevator::retract)));

    autoChooser.addOption("Drive out",
        new SequentialCommandGroup(
            new InstantCommand(algaeArm::retract),
            new InstantCommand(elevator::retract),
            new RunCommand(() -> drivetrain.inputDrivingX_Y(
                MetersPerSecond.of(-1), MetersPerSecond.of(0), RadiansPerSecond.of(0))).withTimeout(2),

            new RunCommand(drivetrain::stop).withTimeout(1)));

    SmartDashboard.putData("Auto chooser", autoChooser);
  }

  private void configureBindings() {
    // CommandScheduler.getInstance().getActiveButtonLoop().clear();
    // driverController.A.onTrue(moveToDistanceApriltag);

    teleDrive.setNormalTriggerBinds(); // MOST DRIVER CONTROLLS ARE HERE

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
    /*
     * RT alae out
     * RB alae in
     * LT shift
     * PResets same
     * A coral intake
     * B coral outtake
     */
    /** Shift == coral mode */
    Trigger Shift = opCont.LeftTrigger;
    Trigger notShift = Shift.negate();
    opCont.RightTrigger
        .onTrue(new InstantCommand(algaeClaw::outtake))
        .onFalse(new InstantCommand(algaeClaw::stop));
    opCont.RightBumper
        .onTrue(new InstantCommand(algaeClaw::intake));

    // coral mode, with trigger
    opCont.PovUp.and(Shift)
        .onTrue(new InstantCommand(elevator::positionL4))
        .onTrue(new InstantCommand(algaeArm::retract));
    opCont.PovRight.and(Shift)
        .onTrue(new InstantCommand(elevator::positionL3))
        .onTrue(new InstantCommand(algaeArm::retract));
    opCont.PovLeft.and(Shift)
        .onTrue(new InstantCommand(elevator::positionL2))
        .onTrue(new InstantCommand(algaeArm::retract));
    opCont.PovDown.and(Shift)
        .onTrue(new InstantCommand(elevator::positionL1))
        .onTrue(new InstantCommand(algaeArm::retract));

    opCont.LeftStickPress.and(Shift)
        .onTrue(new SequentialCommandGroup(
            new WaitCommand(0.1),
            new InstantCommand(elevator::retract)))
        .onTrue(new InstantCommand(algaeArm::retract));

    // algae mode, no trigger
    opCont.PovUp.and(notShift)
        .onTrue(new InstantCommand(elevator::positionBarge))
        .onTrue(new InstantCommand(algaeArm::positionBarge));
    opCont.PovRight.and(notShift)
        .onTrue(new InstantCommand(elevator::positionAlgaeHigh))
        .onTrue(new InstantCommand(algaeArm::positionOut));
    opCont.PovLeft.and(notShift)
        .onTrue(new InstantCommand(elevator::positionAlgaeLow))
        .onTrue(new InstantCommand(algaeArm::positionOut));
    opCont.PovDown.and(notShift)
        .onTrue(new InstantCommand(elevator::positionAlgaeGround))
        .onTrue(new InstantCommand(algaeArm::positionGroundIntake));

    opCont.LeftStickPress.and(notShift)
        .onTrue(new SequentialCommandGroup(
            new WaitCommand(0.1),
            new InstantCommand(elevator::retract)))
        .onTrue(new InstantCommand(algaeArm::retract));

    opCont.LeftBumper.and(notShift)
        .onTrue(new InstantCommand(elevator::positionProccessor))
        .onTrue(new InstantCommand(algaeArm::positionOut));

    // opCont.A
    // .onTrue(coralManipulator.comIntake());
    opCont.A.onTrue(coralManipulator.comIntake());
    opCont.B.onTrue(coralManipulator.comOuttake());

    opCont.Y.onTrue(new InstantCommand(algaeArm::retract));

    opCont.Options // THE ONE LABELED OPTIONS
        .onTrue(new InstantCommand(() -> algaeArm._setAngle(Radian.of(0))));
    opCont.LeftBumper.and(opCont.Options) // THE ONE LABELED SHARE
        .and(Shift)
        .onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> elevator.setPower(-0.1, true)),
            new WaitUntilCommand(elevator.getCustomSensor()),
            new InstantCommand(elevator::stop),
            new InstantCommand(elevator::holdPosition)));

    opCont.Options.onTrue(new InstantCommand(() -> algaeArm.getEncoder(0).setPosition(0)));

    opCont.LeftBumper.and(Shift); // Manual smart control

    if (drivCont.isJoystick) {
      drivCont.buttons[12]
          .onTrue(new InstantCommand(elevator::retract))
          .onTrue(new InstantCommand(algaeArm::retract));

      drivCont.buttons[10] // Right one
          .onTrue(new InstantCommand(() -> elevator
              ._setPosition(Constants.Elevator.positionBoundsMin)));

      // drivCont.buttons[11]
      // .whileTrue(new TrajectoryMoveCommand(generator, holoDrive, true));
    } else {
      // driver xbox
      drivCont.A
          .onTrue(new InstantCommand(elevator::retract))
          .onTrue(new InstantCommand(algaeArm::retract));

      // falling button
      final double emergencyMult = 2;
      drivCont.Y
          .onTrue(new InstantCommand(elevator::retract))
          .onTrue(new InstantCommand(algaeArm::retract))
          .onTrue(new InstantCommand(() -> elevator.setCurrentLimit(-1)))
          .onTrue(new InstantCommand(
              () -> elevator.getPIDConfig().maxLinearAcceleration = elevator
                  .getPIDConfig().maxLinearAcceleration
                  .times(emergencyMult)))
          .onTrue(new InstantCommand(
              () -> elevator.getPIDConfig().maxLinearVelocity = elevator.getPIDConfig().maxLinearVelocity
                  .times(emergencyMult)))
          .onFalse(new InstantCommand(
              () -> elevator.getPIDConfig().maxLinearAcceleration = Constants.Elevator.maxAcceleration))
          .onFalse(new InstantCommand(
              () -> elevator.getPIDConfig().maxLinearVelocity = Constants.Elevator.maxVelocity))
          .onFalse(new InstantCommand(() -> elevator.setCurrentLimit(Constants.Elevator.amps)));
      drivCont.B
          .onTrue(new InstantCommand(algaeArm::retract));
      // drivCont.X
      // .onTrue(new PrintCommand("Trajectory!!!"))
      // .whileTrue(new MoveToDistanceApriltag(drivetrain, holoDrive, photon, 0.1, 0,
      // 0));
      // .whileTrue();
      // drivCont.X
      // .whileTrue(new TrajectoryMoveCommand(generator, holoDrive, true));
        if (false) {
      drivCont.PovUp
          .whileTrue(new RepeatCommand(
              new SequentialCommandGroup(
                  new InstantCommand(
                      () -> elevator.moveToPosition(
                          elevator.getPosition()
                              .plus(Inches.of(1)))),
                  new WaitCommand(0.1))));
      drivCont.PovDown
          .onTrue(new RepeatCommand(
              new SequentialCommandGroup(
                  new InstantCommand(
                      () -> elevator.moveToPosition(
                          elevator.getPosition()
                              .minus(Inches.of(
                                  1)))),
                  new WaitCommand(0.1))));
        }
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}