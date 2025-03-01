package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meters;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.SyncedLibraries.ManipulatorBaseSysID;
import frc.robot.SyncedLibraries.SystemBases.PositionManipulatorBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.ManipulatorFFDistanceCommand;
import frc.robot.SyncedLibraries.SystemBases.Utils.PIDConfig;

public class Elevator extends PositionManipulatorBase {
  public ManipulatorBaseSysID sysID;

  public Elevator() {
    super(new PIDConfig().set(Constants.Elevator.P, Constants.Elevator.I, Constants.Elevator.D,
        Constants.Elevator.S, Constants.Elevator.V, Constants.Elevator.A, Constants.Elevator.G,
        Constants.Elevator.maxVelocity, Constants.Elevator.maxAcceleration),
        ManipulatorFFDistanceCommand.FeedForwardType.Elevator);
    addMotors(new SparkMax(Constants.Wirings.elevatorMotor1, SparkMax.MotorType.kBrushless),
        new SparkMax(Constants.Wirings.elevatorMotor2, SparkMax.MotorType.kBrushless));
    resetMotors();
    invertSpecificMotors(true, 1);
    setBrakeMode(true);
    setBreakerMaxAmps(40);
    setCurrentLimit(Constants.Elevator.amps);
    setPositionMultiplier(Constants.Elevator.positionMultiplier);
    setPositionBounds(Constants.Elevator.positionBoundsMin,
        Constants.Elevator.positionBoundsMax);
    home().schedule();

    customSensor = getMotor(0).getForwardLimitSwitch()::isPressed;
  }

  public void retract() {
    moveToPosition(0);
  }

  /** If falling, send it */
  public void RETRACT() {
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
    SmartDashboard.putNumber("Elevator target position", ((ManipulatorFFDistanceCommand) moveCommand).getController().getSetpoint().position);
    SmartDashboard.putNumber("Elevator Setpoint", ((ManipulatorFFDistanceCommand) moveCommand).getController().getGoal().position);
    SmartDashboard.putNumber("Elevator current position", getPosition().in(Meters));
  }
}
