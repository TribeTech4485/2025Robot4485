// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Autos;
import frc.robot.Commands.DumbAprilMove;
import frc.robot.Commands.MoveToDistanceApriltag;
import frc.robot.Commands.MoveToDistanceCommand;
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
import frc.robot.SyncedLibraries.SystemBases.PathPlanning.TrajectoryMoveCommand;
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
  LEDS leds = new LEDS(elevator);

  TeleDrive teleDrive = new TeleDrive(drivetrain, controllers, elevator, algaeArm, algaeClaw, coralManipulator);
  HolonomicDriveController holoDrive = new HolonomicDriveController(
      new PIDController(Constants.Swerve.Movement.Holonomic.xP,
          Constants.Swerve.Movement.Holonomic.xI, Constants.Swerve.Movement.Holonomic.xD),
      new PIDController(Constants.Swerve.Movement.Holonomic.yP,
          Constants.Swerve.Movement.Holonomic.yI, Constants.Swerve.Movement.Holonomic.yD),
      drivetrain.getTurnController());

  // BackgroundTrajectoryGenerator generator = new BackgroundTrajectoryGenerator(
  // new Pose2d(),
  // new Pose2d(Inches.of(140), Inches.of(0), Rotation2d.fromDegrees(0)),
  // List.of(new Translation2d(0.1, 0)),
  // MetersPerSecond.of(0.75), MetersPerSecondPerSecond.of(5));
  // BackgroundTrajectoryGenerator generator2 = new BackgroundTrajectoryGenerator(
  // new Pose2d(),
  // new Pose2d(Inches.of(-40), Inches.of(0), Rotation2d.fromDegrees(0)),
  // List.of(new Translation2d(-20, 0)),
  // MetersPerSecond.of(1), MetersPerSecondPerSecond.of(5));

  // BackgroundTrajectoryGenerator move1meterTrajectory = new
  // BackgroundTrajectoryGenerator(
  // new Pose2d(),
  // new Pose2d(Meters.of(1), Inches.of(0), Rotation2d.fromDegrees(0)),
  // List.of(new Translation2d(0.1, 0)),
  // MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(1));

  // BackgroundTrajectoryGenerator move1footTrajectory = new
  // BackgroundTrajectoryGenerator(
  // new Pose2d(),
  // new Pose2d(Feet.of(5), Inches.of(0), Rotation2d.fromDegrees(0)),
  // List.of(new Translation2d(0.1, 0)),
  // MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(1));

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  // MoveToDistanceApriltag moveToDistanceApriltag = new
  // MoveToDistanceApriltag(drivetrain, photon, 1, 0, 0);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();

    System.out.print("Creating autos... ");
    autoChooser.addOption("Old Coral Place", Autos.coralPlace(drivetrain, elevator, algaeArm, coralManipulator));
    autoChooser.addOption("Drive out", Autos.driveOut(drivetrain, elevator, algaeArm));
    autoChooser.addOption("NOOO Center Coral and Algae",
        Autos.centerCoralAndAlgae(drivetrain, elevator, algaeArm, coralManipulator));
    autoChooser.setDefaultOption("April place",
        Autos.aprilAuto(drivetrain, photon, teleDrive, elevator, algaeArm, coralManipulator, algaeClaw));

    SmartDashboard.putData("Auto chooser", autoChooser);
    System.out.println("Done");
  }

  private void configureBindings() {
    teleDrive.setNormalTriggerBinds(); // MOST DRIVER CONTROLLS ARE HERE

    // drivCont.ESTOPCondition.onTrue(new InstantCommand(Estopable::KILLIT));
    /** Shift == coral mode */
    Trigger Shift = opCont.LeftTrigger;
    Trigger notShift = Shift.negate();
    opCont.RightTrigger.and(notShift)
        .onTrue(new InstantCommand(algaeClaw::outtake))
        .onFalse(new InstantCommand(algaeClaw::stop));
    opCont.RightTrigger.and(Shift)
        .onTrue(new InstantCommand(algaeClaw::plop))
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

    // retract all
    opCont.LeftStickPress
        .onTrue(new SequentialCommandGroup(
            new WaitCommand(0.1),
            new InstantCommand(elevator::retract)))
        .onTrue(new InstantCommand(algaeArm::retract));

    opCont.Y.onTrue(new InstantCommand(algaeArm::retract));
    opCont.RightStickPress
        .onTrue(new InstantCommand(elevator::holdPosition))
        .onTrue(new InstantCommand(algaeArm::holdPosition));

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

    opCont.LeftBumper.and(notShift)
        .onTrue(new InstantCommand(elevator::positionProccessor))
        .onTrue(new InstantCommand(algaeArm::positionOut));

    // opCont.A
    // .onTrue(coralManipulator.comIntake());
    opCont.A.onTrue(coralManipulator.comIntake());
    opCont.B.onTrue(coralManipulator.comOuttake());

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
      final double emergencyMult = 4;
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
      drivCont.B.onTrue(new InstantCommand(algaeArm::retract));
      // drivCont.X
      // .onTrue(new PrintCommand("Trajectory!!!"))
      // .whileTrue(new MoveToDistanceApriltag(drivetrain, holoDrive, photon, 0.1, 0,
      // 0));
      // .whileTrue();
      // drivCont.X
      // .whileTrue(new TrajectoryMoveCommand(move1meterTrajectory, holoDrive,
      // drivetrain, true));

      // drivCont.Start.onTrue(new InstantCommand(drivetrain::resetDriveEncoders));
      drivCont.Start.onTrue(new InstantCommand(() -> Estopable.KILLIT(false)));

      // drivCont.PovLeft
      // .whileTrue(new MoveToDistanceApriltag(drivetrain, holoDrive, photon,
      // Inches.of(-30), Inches.of(-12), Degrees.of(0)));
      // drivCont.PovRight
      // .whileTrue(new MoveToDistanceApriltag(drivetrain, holoDrive, photon,
      // Inches.of(-30), Inches.of(12), Degrees.of(0)));
      // drivCont.PovUp
      // .whileTrue(new MoveToDistanceApriltag(drivetrain, holoDrive, photon,
      // Meters.of(-0.75), Inches.of(0), Degrees.of(180)));
      // drivCont.PovDown
      // .whileTrue(new TrajectoryMoveCommand(move1footTrajectory, holoDrive,
      // drivetrain, true));

      // drivCont.PovUp
      // .whileTrue(new MoveToDistanceCommand(drivetrain, Meters.of(2),
      // Meters.of(0), Radians.of(0),
      // MetersPerSecond.of(1), MetersPerSecondPerSecond.of(2.5)));

      // drivCont.PovLeft
      // .whileTrue(new MoveToDistanceCommand(drivetrain, Meters.of(0),
      // Meters.of(-2), Radians.of(0),
      // MetersPerSecond.of(1), MetersPerSecondPerSecond.of(2.5)));

      // drivCont.PovRight.debounce(0.2)
      // .onTrue(new InstantCommand(teleDrive::disable))
      // .onFalse(new InstantCommand(teleDrive::enable))
      // .onTrue(new DumbAprilMove(drivetrain, photon,
      // Feet.of(3), Feet.of(0), Radians.zero()));
      // drivCont.PovRight
      // .whileTrue(new MoveToDistanceCommand(drivetrain, Meters.of(2),
      // Meters.of(2), Radians.of(0),
      // MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(2.5)));

      // drivCont.PovDown
      // .whileTrue(Autos.aprilAuto(drivetrain, photon, teleDrive, elevator, algaeArm,
      // coralManipulator));
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public TeleDrive getTeleDrive() {
    return teleDrive;
  }
}