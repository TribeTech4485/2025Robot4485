package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.ManipulatorFFMoveCommand;

public class AlgaeArm extends ManipulatorBase {

  // 0 is straight up, 90 is perpendicular to the ground
  public AlgaeArm() {
    addMotors(new SparkMax(Constants.Wirings.algaeArmMotor, SparkMax.MotorType.kBrushless));
    setBrakeMode(true);
    setCurrentLimit(Constants.AlgaeArm.currentLimit);
    setPositionMultiplier(Constants.AlgaeArm.positionMultiplier);
    // 0 is straight out, pi/2 is straight up
    setPositionBounds(Constants.AlgaeArm.positionBoundsMin, Constants.AlgaeArm.positionBoundsMax);

    setPositionPID(new ManipulatorFFMoveCommand(this, 1.4, 0, Constants.AlgaeArm.P, Constants.AlgaeArm.I,
        Constants.AlgaeArm.D, ManipulatorFFMoveCommand.FeedForwardType.Arm, Constants.AlgaeArm.S,
        Constants.AlgaeArm.V, Constants.AlgaeArm.G, Constants.AlgaeArm.A,
        Constants.AlgaeArm.maxVelocity, Constants.AlgaeArm.maxAcceleration));
    // TODO: if elevator is going down, arm should go up
  }

  @Override
  public Command test() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> moveToPosition(90, false)),
        new WaitUntilCommand(() -> isAtPosition()),
        new InstantCommand(() -> moveToPosition(0, false)));
  }

  @Override
  public void ESTOP() {
    fullStop();
    setBrakeMode(true);
  }

  public void retract() {
    moveToPosition(0);
  }

  @Override
  public void moveToPosition(double position) {
    moveToPosition(0, false);
  }
}
