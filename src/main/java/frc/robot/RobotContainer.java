// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Subsystems.Swerve.*;
import frc.robot.SyncedLibraries.BasicFunctions;
import frc.robot.SyncedLibraries.SystemBases.ControllerBase;

public class RobotContainer {
  Drivetrain drivetrain = new Drivetrain();
  // ControllerBase controller = new ControllerBase(0, false, false, true);
  CommandJoystick cont = new CommandJoystick(0);
  Joystick controller = cont.getHID();

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    // controller.buttons[2].get().onTrue(
    //     new InstantCommand(() -> drivetrain.enableXLock())).onFalse(
    //         new InstantCommand(() -> drivetrain.disableXLock()));

    // cont.get(2).onTrue(
    // new InstantCommand(() -> drivetrain.enableXLock())).onFalse(
    // new InstantCommand(() -> drivetrain.disableXLock()));

    // controller.ESTOPCondition.get().onTrue(
    //     new InstantCommand(() -> BasicFunctions.KILLIT()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
