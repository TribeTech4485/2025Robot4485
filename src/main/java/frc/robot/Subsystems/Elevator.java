package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.SyncedLibraries.ManipulatorBaseSysID;
import frc.robot.SyncedLibraries.SystemBases.PositionManipulatorBase;
import frc.robot.SyncedLibraries.SystemBases.Utils.ManipulatorFFDistanceCommand;
import frc.robot.SyncedLibraries.SystemBases.Utils.PIDConfig;

public class Elevator extends PositionManipulatorBase {
  public ManipulatorBaseSysID sysID;
  final Trigger lowLimit;

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
    // home().schedule();
    persistMotorConfig();
    _setPosition(minPosition);

    customSensor = getMotor(0).getForwardLimitSwitch()::isPressed;
    lowLimit = new Trigger(customSensor);
    lowLimit.onTrue(new InstantCommand(() -> _setPosition(minPosition)));
  }

  public void retract() {
    moveToPosition(minPosition);
  }

  /** If falling, send it */
  public void RETRACT() {
    setCurrentLimit(40);
    retract();
  }

  // Algae
  public void positionAlgaeGround() {
    moveToPosition(Feet.of(2));
  }

  public void positionAlgaeLow() {
    moveToPosition(Feet.of(3.25));
  }

  public void positionAlgaeHigh() {
    moveToPosition(Feet.of(4.5));
  }

  // Coral
  public void positionL1() {
    moveToPosition(Feet.of(2));
  }

  public void positionL2() {
    moveToPosition(Inches.of(24 + 8));
  }

  public void positionL3() {
    moveToPosition(Inches.of(48));
  }

  public void positionL4() {
    positionTop();
  }

  public void positionTop() {
    moveToPosition(maxPosition);
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
        new InstantCommand(() -> _setPosition(minPosition)),
        new InstantCommand(() -> fullStop()),
        new InstantCommand(() -> setCurrentLimit(Constants.Elevator.amps)),
        new InstantCommand(() -> moveToPosition(minPosition)),
        new InstantCommand(() -> getMoveCommand().setEndOnTarget(false)));
  }

  @Override
  public Command test() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> moveToPosition(minPosition)),
        new WaitUntilCommand(() -> isAtPosition()),
        new InstantCommand(() -> moveToPosition(maxPosition)));
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
    SmartDashboard.putNumber("Elevator target position",
        ((ManipulatorFFDistanceCommand) moveCommand).getController().getSetpoint().position);
    SmartDashboard.putNumber("Elevator Setpoint",
        ((ManipulatorFFDistanceCommand) moveCommand).getController().getGoal().position);
    SmartDashboard.putNumber("Elevator current position", getPosition().in(Meters));
    SmartDashboard.putNumber("Line Voltage", getMotor(0).getBusVoltage());
  }
}
