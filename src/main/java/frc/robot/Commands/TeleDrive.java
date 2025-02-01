package frc.robot.Commands;

import frc.robot.SyncedLibraries.SystemBases.ControllerBase;
import frc.robot.SyncedLibraries.SystemBases.TeleDriveCommandBase;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;

public class TeleDrive extends TeleDriveCommandBase {

  public TeleDrive(SwerveDriveBase driveTrain, ControllerBase mainDriver, ControllerBase secondaryDriver) {
    super(driveTrain, mainDriver, secondaryDriver);
  }

}
