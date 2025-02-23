// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SyncedLibraries.SystemBases.Estopable;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorBase;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command testCommand;
  private int counter = 0;

  /*
   * Wanted controls:
   * Trigger (0): Full drive speed (disable rpm based speed control)
   * 2: Disable driving and joystick controls angle
   * 3/4: Enable/disable field oriented driving
   * 5: Reset gyro
   * Throttle: Change max speed
   * POV: change rotation center
   */

  public RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (counter++ % 100 == 0) {
      System.out.println(m_robotContainer.testConfig.P);
    }
  }

  @Override
  public void disabledInit() {
    if (DriverStation.isEStopped()) {
      Estopable.dontAllowFullEstop();
      Estopable.KILLIT();
    }
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
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    ManipulatorBase[] manipulators = ManipulatorBase.getAllManipulators();
    Command[] tests = new Command[manipulators.length + 1];
    tests[0] = new PrintCommand("Starting Tests");
    for (int i = 1; i < manipulators.length; i++) {
      Command test = manipulators[i].test();
      if (test != null) {
        tests[i] = test;
      } else {
        tests[i] = new PrintCommand("No test for " + manipulators[i].getName());
      }
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
