package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorBase;

public class AlgaeArm extends ManipulatorBase {

    // 0 is straight up, 90 is perpendicular to the ground
    public AlgaeArm() {
        addMotors(new SparkMax(Constants.Wirings.algaeArmMotor, SparkMax.MotorType.kBrushless));
        setBrakeMode(true);
        setCurrentLimit(Constants.AlgaeArm.currentLimit);

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
}
