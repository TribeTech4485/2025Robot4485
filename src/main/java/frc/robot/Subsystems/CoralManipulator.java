package frc.robot.Subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorBase;
import frc.robot.Constants;

public class CoralManipulator extends ManipulatorBase {
  ColorSensorV3 colorSensor;
  boolean sensed = false;
  int counter = 0;

  public CoralManipulator() {
    addMotors(new SparkMax(Constants.Wirings.coralManipulatorMotor, SparkMax.MotorType.kBrushless));
    setBrakeMode(true);
    setCurrentLimit(Constants.CoralManipulator.currentLimit);
    colorSensor = new ColorSensorV3(Port.kMXP);
    customSensor = () -> sensed;
  }

  @Override
  public Command test() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> setPower(1)),
        new WaitCommand(1),
        new InstantCommand(() -> setPower(-1)),
        new WaitCommand(1),
        new InstantCommand(() -> setPower(0)));
  }

  public void intake() {
    setPower(1);

  }

  public void outtake() {
    setPower(-1);
  }

  public Command comIntake() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> intake()),
        new WaitUntilCommand(customSensor),
        new WaitUntilCommand(() -> !customSensor.getAsBoolean()),
        new InstantCommand(() -> setPower(-0.5)),
        new WaitCommand(0.1),
        new InstantCommand(() -> stop()))
        .withTimeout(5).andThen(
            new InstantCommand(() -> stop()));
  }

  public Command comOuttake() {
    return new SequentialCommandGroup(new InstantCommand(() -> outtake()),
        new WaitCommand(2),
        new InstantCommand(() -> stop()));
  }

  @Override
  public void ESTOP() {
    setBrakeMode(false);
    fullStop();
  }

  @Override
  public void periodic() {
    super.periodic();
    if (counter++ == 20) {
      counter = 0;
      sensed = colorSensor.getProximity() > 150;
    }

    // SmartDashboard.putNumber("Coral sensor distance", sensor.getProximity());
    // SmartDashboard.putBoolean("Coral sened", customSensor.getAsBoolean());
  }
}
