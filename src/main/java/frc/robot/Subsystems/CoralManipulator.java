package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorBase;
import frc.robot.Constants;

public class CoralManipulator extends ManipulatorBase {

    public CoralManipulator() {
        addMotors(new SparkMax(Constants.Wirings.coralManipulatorMotor, SparkMax.MotorType.kBrushless));
        setBrakeMode(true);
        setCurrentLimit(Constants.CoralManipulator.currentLimit);
    }

    // TODO: Basic intake and outtake methods for the smooth brain folk

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
        // TODO: your turn...
    }

}
