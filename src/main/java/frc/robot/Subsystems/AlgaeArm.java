package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.SyncedLibraries.SystemBases.AngleManipulatorBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.ManipulatorFFAngleCommand;
import frc.robot.SyncedLibraries.SystemBases.Utils.PIDConfig;

public class AlgaeArm extends AngleManipulatorBase {
  final Elevator elevator;

  public AlgaeArm(Elevator elevator) {
    super(
        new PIDConfig().set(Constants.AlgaeArm.P, Constants.AlgaeArm.I, Constants.AlgaeArm.D, Constants.AlgaeArm.S,
            Constants.AlgaeArm.V, Constants.AlgaeArm.A, Constants.AlgaeArm.G, Constants.AlgaeArm.maxVelocity,
            Constants.AlgaeArm.maxAcceleration),
        ManipulatorFFAngleCommand.FeedForwardType.Arm);
    addMotors(new SparkMax(Constants.Wirings.algaeArmMotor, SparkMax.MotorType.kBrushless));
    resetMotors();
    setBrakeMode(false);
    setCurrentLimit(Constants.AlgaeArm.amps);
    setPositionMultiplier(Constants.AlgaeArm.positionMultiplier);
    invertSpecificMotors(false, 0);
    // 0 is straight out, positive is up
    setAngleBounds(Constants.AlgaeArm.positionBoundsMin, Constants.AlgaeArm.positionBoundsMax);
    // home().schedule();
    getMoveCommand().setEndOnTarget(false);
    customSensor = getMotor(0).getForwardLimitSwitch()::isPressed;
    this.elevator = elevator;
  }

  @Override
  public Command test() {
    return new SequentialCommandGroup(
        new InstantCommand(this::retract),
        new WaitUntilCommand(this::isAtPosition),
        new InstantCommand(() -> moveToPosition(0)),
        new WaitUntilCommand(this::isAtPosition),
        new InstantCommand(this::retract));
  }

  @Override
  public Command home() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> setCurrentLimit(1)),
        new InstantCommand(() -> setPower(0.1)),
        new WaitUntilCommand(customSensor),
        new InstantCommand(() -> _setAngle(Degrees.of(110))),
        new InstantCommand(this::fullStop),
        new InstantCommand(() -> setCurrentLimit(Constants.AlgaeArm.amps)),
        new InstantCommand(this::retract),
        new InstantCommand(() -> getMoveCommand().setEndOnTarget(false)));
  }

  @Override
  public void ESTOP() {
    fullStop();
    setBrakeMode(true);
  }

  public void retract() {
    moveToPosition(60);
  }

  @Override
  public void moveToPosition(Angle position) {
    moveToPosition(position, false);
  }

  public void moveToPosition(double degrees) {
    moveToPosition(Degrees.of(degrees));
  }

  public void elevatorCheck() {
    Distance elevatorPosition = elevator.getPosition();
    // TODO: find the correct values for elevator check
    if (elevatorPosition.compareTo(Feet.of(0.5)) < 0
        && getAngle().compareTo(Degrees.of(0)) < 0) {
      // if elevator is below 0.5ft and arm is below level
      retract();
    }

    if (elevatorPosition.compareTo(Meters.of(1.5)) > 0
        && getAngle().compareTo(Degrees.of(50)) > 0) {
      // if elevator is above 1.5m and arm is above 50 degrees
      moveToPosition(Degrees.of(45));
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber(getName() + " Setpoint Angle (deg)",
        ((ManipulatorFFAngleCommand) getMoveCommand()).getSetpoint().in(Degrees));
    elevatorCheck();
  }
}
