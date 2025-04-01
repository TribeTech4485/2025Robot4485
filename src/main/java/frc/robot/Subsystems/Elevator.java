package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
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
    setBreakerMaxAmps(40);
    addMotors(new SparkMax(Constants.Wirings.elevatorMotor1, SparkMax.MotorType.kBrushless),
        new SparkMax(Constants.Wirings.elevatorMotor2, SparkMax.MotorType.kBrushless));
    resetMotors();
    invertSpecificMotors(true, 1);
    setBrakeMode(true);
    setCurrentLimit(Constants.Elevator.amps);
    setPositionMultiplier(Constants.Elevator.positionMultiplier);
    setSpeedMultiplier(Constants.Elevator.positionMultiplier / 50);
    setPositionBounds(Constants.Elevator.positionBoundsMin,
        Constants.Elevator.positionBoundsMax);
    // home().schedule();
    persistMotorConfig();

    customSensor = () -> getMotor(0).getReverseLimitSwitch().isPressed()
        || getMotor(1).getReverseLimitSwitch().isPressed();
    lowLimit = new Trigger(customSensor);
    lowLimit.onTrue(new InstantCommand(() -> _setPosition(minPosition)));
    if (customSensor.getAsBoolean()) {
      _setPosition(minPosition);
    }

    Robot.onInits.add(
        new ConditionalCommand(new InstantCommand(this::holdPosition),
            new InstantCommand(),
            () -> !getMoveCommand().isScheduled()));
  }

  public void retract() {
    moveToPosition(minPosition.plus(Inches.of(0)));
  }

  /** If falling, send it */
  public void RETRACT() {
    setCurrentLimit(-1);
    retract();
  }

  public void positionSource() {
    moveToPosition(Feet.of(1.25));
  }

  // Algae
  public void positionAlgaeGround() {
    moveToPosition(Feet.of(1.75));
  }

  public void positionProccessor() {
    moveToPosition(Inches.of(20));
  }

  public void positionAlgaeLow() {
    moveToPosition(Inches.of(31));
  }

  public void positionAlgaeHigh() {
    moveToPosition(Inches.of(43));
  }

  // Coral
  public void positionL1() {
    moveToPosition(Feet.of(2));
  }

  public void positionL2() {
    moveToPosition(Inches.of(30));
  }

  public void positionL3() {
    moveToPosition(Inches.of(43));
  }

  public void positionL4() {
    moveToPosition(Inches.of(63));
  }

  public void positionBarge() {
    moveToPosition(Inches.of(78));
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

  public void holdPosition() {
    moveToPosition(getPosition().plus(getVelocity().times(Seconds.of(0.1))));
  }

  public void adjustBy(Distance distance) {
    moveToPosition(getPosition().plus(distance));
  }

  public double getPosPercent() {
    return getPosition().minus(minPosition).div(maxPosition.minus(minPosition)).baseUnitMagnitude();

  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("ElePow", getAbsVoltage());
    SmartDashboard.putNumber("Elevator target position",
        ((ManipulatorFFDistanceCommand) moveCommand).getController().getSetpoint().position);
    SmartDashboard.putNumber("Elevator Setpoint",
        ((ManipulatorFFDistanceCommand) moveCommand).getController().getGoal().position);
    SmartDashboard.putNumber("Elevator current position", getPosition().in(Meters));
    SmartDashboard.putNumber("Line Voltage", getMotor(0).getBusVoltage());
    SmartDashboard.putBoolean("Elevator low limit", lowLimit.getAsBoolean());

    SmartDashboard.putNumber("Elevator target speed",
        ((ManipulatorFFDistanceCommand) moveCommand).getController().getSetpoint().velocity);
    SmartDashboard.putNumber("Elevator current speed", getVelocity().in(MetersPerSecond));

    SmartDashboard.putNumber("Elevator height percent", getPosPercent());
  }
}
