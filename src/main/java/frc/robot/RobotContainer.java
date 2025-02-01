// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.MoveToDistanceApriltag;
import frc.robot.Commands.TeleDrive;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.AlgaeClaw;
import frc.robot.Subsystems.PhotonVision;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.SyncedLibraries.Controllers;
import frc.robot.SyncedLibraries.SystemBases.ControllerBase;
import frc.robot.SyncedLibraries.SystemBases.Estopable;

public class RobotContainer {
  Controllers controllers = new Controllers();
  ControllerBase driverController = controllers.Zero;
  ControllerBase operatorController = controllers.One;
  Drivetrain drivetrain = new Drivetrain();
  PhotonVision photon = new PhotonVision();
  AlgaeClaw algaeManipulator = new AlgaeClaw();
  Elevator elevator = new Elevator();
  TeleDrive teleDrive = new TeleDrive(drivetrain, driverController, operatorController);

  MoveToDistanceApriltag moveToDistanceApriltag = new MoveToDistanceApriltag(drivetrain, photon, 1, 0, 0);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
    Estopable.dontAllowFullEstop();
  }

  private void configureBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    // driverController.A.onTrue(moveToDistanceApriltag);
    teleDrive.setNormalTriggerBinds();

    driverController.PovUp.and(driverController.A)
        .whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driverController.PovUp.and(driverController.B)
        .whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    driverController.PovUp.and(driverController.X)
        .whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driverController.PovUp.and(driverController.Y)
        .whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    driverController.PovDown.and(driverController.A)
        .whileTrue(drivetrain.dynamicSysID(SysIdRoutine.Direction.kForward));
    driverController.PovDown.and(driverController.B)
        .whileTrue(drivetrain.dynamicSysID(SysIdRoutine.Direction.kReverse));

    driverController.PovDown.and(driverController.X)
        .whileTrue(drivetrain.quasistaticSysID(SysIdRoutine.Direction.kForward));
    driverController.PovDown.and(driverController.Y)
        .whileTrue(drivetrain.quasistaticSysID(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
