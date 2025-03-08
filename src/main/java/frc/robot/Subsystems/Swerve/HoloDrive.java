package frc.robot.Subsystems.Swerve;

import frc.robot.SyncedLibraries.SystemBases.PathPlanning.HolonomicDriveBase;
import frc.robot.SyncedLibraries.SystemBases.Swerve.SwerveDriveBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.PIDConfig;
import frc.robot.Constants.Swerve.Movement.Holonomic;

public class HoloDrive extends HolonomicDriveBase {
  public HoloDrive(SwerveDriveBase driveBase) {
    super(driveBase,
        new PIDConfig().set(Holonomic.xP, Holonomic.xI, Holonomic.xD),
        new PIDConfig().set(Holonomic.yP, Holonomic.yI, Holonomic.yD));
  }
}