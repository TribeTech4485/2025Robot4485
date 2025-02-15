package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.SyncedLibraries.SystemBases.AngleManipulatorBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.ManipulatorFFAngleCommand;

public class AlgaeArm extends AngleManipulatorBase {

  // 0 is straight up, 90 is perpendicular to the ground
  public AlgaeArm() {
    addMotors(new SparkMax(Constants.Wirings.algaeArmMotor, SparkMax.MotorType.kBrushless));
    setBrakeMode(true);
    setCurrentLimit(Constants.AlgaeArm.currentLimit);
    setPositionMultiplier(Constants.AlgaeArm.positionMultiplier);
    // 0 is straight out, pi/2 is straight up
    setAngleBounds(Constants.AlgaeArm.positionBoundsMin, Constants.AlgaeArm.positionBoundsMax);

    setPositionPID(new ManipulatorFFAngleCommand(this, getAngle(), Degrees.of(0), Constants.AlgaeArm.P,
        Constants.AlgaeArm.I,
        Constants.AlgaeArm.D, ManipulatorFFAngleCommand.FeedForwardType.Arm, Constants.AlgaeArm.S,
        Constants.AlgaeArm.V, Constants.AlgaeArm.G, Constants.AlgaeArm.A,
        Constants.AlgaeArm.maxVelocity, Constants.AlgaeArm.maxAcceleration));
    // TODO: if elevator is going down, arm should go up
  }

  @Override
  public Command test() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> moveToPosition(90)),
        new WaitUntilCommand(() -> isAtPosition()),
        new InstantCommand(() -> moveToPosition(0)));
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
  public void moveToPosition(Angle position) {
    moveToPosition(position, false);
  }

  public void moveToPosition(double degrees) {
    moveToPosition(Degrees.of(degrees));
  }
}
