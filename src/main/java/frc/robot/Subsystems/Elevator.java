package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meters;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.SyncedLibraries.ManipulatorBaseSysID;
import frc.robot.SyncedLibraries.SystemBases.PositionManipulatorBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.ManipulatorFFDistanceCommand;

public class Elevator extends PositionManipulatorBase {
  public ManipulatorBaseSysID sysID;

  public Elevator() {
    for (SparkMax motor: getMotors()) {
      motor.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    // TODO: Custom sensor as lower limit switch
    addMotors(new SparkMax(Constants.Wirings.elevatorMotor1, SparkMax.MotorType.kBrushless),
        new SparkMax(Constants.Wirings.elevatorMotor2, SparkMax.MotorType.kBrushless));
    // invertSpecificMotors(true, 0);
    setBrakeMode(true);
    setBreakerMaxAmps(40);
    setCurrentLimit(Constants.Elevator.amps);
    setPositionMultiplier(Constants.Elevator.positionMultiplier);
    setPositionBounds(Constants.Elevator.positionBoundsMin,
        Constants.Elevator.positionBoundsMax);
    home().schedule();
    setPositionPID(
        new ManipulatorFFDistanceCommand(this, Meters.of(0), Meters.of(0), Constants.Elevator.P, Constants.Elevator.I,
            Constants.Elevator.D, ManipulatorFFDistanceCommand.FeedForwardType.Elevator,
            Constants.Elevator.S, Constants.Elevator.V,
            Constants.Elevator.G, Constants.Elevator.A, Constants.Elevator.maxVelocity,
            Constants.Elevator.maxAcceleration));

    customSensor = getMotor(0).getForwardLimitSwitch()::isPressed;

    // sysID = new ManipulatorBaseSysID(this);
  }

  public void retract() {
    moveToPosition(0);
  }

  /** If falling, send it */
  public void RETRACT() {
    // TODO: make it manual overide
    setCurrentLimit(40);
    retract();
  }

  public void positionIntake() {
    moveToPosition(0.1);
  }

  public void positionL1() {
    moveToPosition(0.5);
  }

  public void positionL2() {
    moveToPosition(1);
  }

  public void positionL3() {
    moveToPosition(1.5);
  }

  public void positionL4() {
    moveToPosition(2);
  }

  @Override
  public void ESTOP() {
    setBrakeMode(true);
    fullStop();
  }

  @Override
  public Command home() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> setCurrentLimit(1)),
        new InstantCommand(() -> setPower(-0.1)),
        new WaitUntilCommand(customSensor),
        new InstantCommand(() -> _setPosition(Meters.of(0))),
        new InstantCommand(() -> fullStop()),
        new InstantCommand(() -> setCurrentLimit(Constants.Elevator.amps)),
        new InstantCommand(() -> moveToPosition(0)),
        new InstantCommand(() -> getMoveCommand().setEndOnTarget(false)));
  }

  @Override
  public Command test() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> moveToPosition(Meters.of(0))),
        new WaitUntilCommand(() -> isAtPosition()),
        new InstantCommand(() -> moveToPosition(Meters.of(1.5))));
  }

  @Override
  public void moveToPosition(Distance position) {
    moveToPosition(position, false);
  }

  public void moveToPosition(double meters) {
    moveToPosition(Meters.of(meters));
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
