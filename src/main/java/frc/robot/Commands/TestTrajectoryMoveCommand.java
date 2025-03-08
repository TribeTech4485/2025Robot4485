package frc.robot.Commands;

import frc.robot.SyncedLibraries.SystemBases.PathPlanning.HolonomicDriveBase;
import frc.robot.SyncedLibraries.SystemBases.PathPlanning.TrajectoryMoveCommand;
import frc.robot.SyncedLibraries.SystemBases.Utils.BackgroundTrajectoryGenerator;

public class TestTrajectoryMoveCommand extends TrajectoryMoveCommand {

  public TestTrajectoryMoveCommand(BackgroundTrajectoryGenerator generator, HolonomicDriveBase driveBase) {
    super(generator, driveBase, true);
  }
}
