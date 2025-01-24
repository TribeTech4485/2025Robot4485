// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.LinkedList;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SyncedLibraries.SystemBases.Estopable;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorBase;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command testCommand;

  /*
   * Wanted controls:
   * Trigger (0): Full drive speed (disable rpm based speed control)
   * 2: Disable driving and joystick controls angle
   * 3/4: Enable/disable field oriented driving
   * 5: Reset gyro
   * Throttle: Change max speed
   * POV: change rotation center
   */

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    if (m_robotContainer.driverController.RightTrigger.getAsBoolean()) {
      m_robotContainer.drivetrain.inputDrivingX_Y_A(
          m_robotContainer.driverController.getRightX(),
          -m_robotContainer.driverController.getRightY(),
          Math.atan2(m_robotContainer.driverController.getLeftX(), m_robotContainer.driverController.getLeftY()),
          m_robotContainer.driverController.getPOV());
      // m_robotContainer.drivetrain.inputDrivingX_Y(-joystick.getLeftY(),
      // joystick.getLeftX(), -joystick.getRightX(), joystick.getPOV());
    } else {
      m_robotContainer.drivetrain.inputDrivingX_Y(0, 0, 0);
    }
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    LinkedList<ManipulatorBase> manipulators = new LinkedList<ManipulatorBase>();
    for (Estopable estopable : Estopable.getAllEstopables()) {
      if (estopable instanceof ManipulatorBase) {
        manipulators.add((ManipulatorBase) estopable);
      }
    }
    Command[] tests = new Command[manipulators.size()];
    for (int i = 0; i < manipulators.size(); i++) {
      tests[i] = manipulators.get(i).test();
    }
    testCommand = new SequentialCommandGroup(tests);
    testCommand.schedule();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
    if (testCommand != null) {
      testCommand.cancel();
    }
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
