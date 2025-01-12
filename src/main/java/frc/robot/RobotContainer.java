// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Commands.MoveToDistanceApriltag;
import frc.robot.Subsystems.PhotonVision;
import frc.robot.Subsystems.Swerve.*;
import frc.robot.SyncedLibraries.BasicFunctions;
import frc.robot.SyncedLibraries.Controllers;
import frc.robot.SyncedLibraries.SystemBases.ControllerBase;

public class RobotContainer {
  Controllers controllers = new Controllers(0.05, 0.05);
  ControllerBase driverController = controllers.Zero;
  Drivetrain drivetrain = new Drivetrain();
  PhotonVision photon = new PhotonVision(new PhotonCamera("mainCamera"));
  MoveToDistanceApriltag moveToDistanceApriltag = new MoveToDistanceApriltag(drivetrain, photon, 1, 0, 0);

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    driverController.ESTOPCondition.onTrue(new InstantCommand(BasicFunctions::KILLIT));

    driverController.A.onTrue(moveToDistanceApriltag);

    driverController.RightBumper.toggleOnTrue(
        new StartEndCommand(drivetrain::enableXLock,
            drivetrain::disableXLock));

    // controller.buttons[2].get().onTrue(
    // new InstantCommand(() -> drivetrain.enableXLock())).onFalse(
    // new InstantCommand(() -> drivetrain.disableXLock()));

    // cont.get(2).onTrue(
    // new InstantCommand(() -> drivetrain.enableXLock())).onFalse(
    // new InstantCommand(() -> drivetrain.disableXLock()));

    // controller.ESTOPCondition.get().onTrue(
    // new InstantCommand(() -> BasicFunctions.KILLIT()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
