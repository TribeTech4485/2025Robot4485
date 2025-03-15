package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorBase;

public class AlgaeClaw extends ManipulatorBase {

  public AlgaeClaw() {
    breakerMaxAmps = 30;
    addMotors(new SparkMax(Constants.Wirings.algaeClawMotor, SparkMax.MotorType.kBrushless));
    resetMotors();
    setBrakeMode(true);
    setCurrentLimit(Constants.AlgaeClaw.currentInLimit);
  }

  public void intake() {
    setCurrentLimit(Constants.AlgaeClaw.currentInLimit);
    setPower(1);
  }

  public void outtake() {
    setCurrentLimit(Constants.AlgaeClaw.currentOutLimit);
    setPower(-0.5);
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

  @Override
  public void ESTOP() {
    setBrakeMode(false);
    fullStop();
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
