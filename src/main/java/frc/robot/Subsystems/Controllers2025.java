package frc.robot.Subsystems;

import frc.robot.SyncedLibraries.Controllers;
import frc.robot.SyncedLibraries.SystemBases.ControllerBase;

public class Controllers2025 extends Controllers {
  @Override
  public void fullUpdate() {
    Zero = new ControllerBase(0, false, false, true);
    One = new ControllerBase(1, false, true, false);
    Two = new ControllerBase(-1);
    Three = new ControllerBase(-1);
    Four = new ControllerBase(-1);
    Five = new ControllerBase(-1);
  }
}
