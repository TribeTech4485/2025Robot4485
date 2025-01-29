package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorBase;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorFFMoveCommand;

public class Elevator extends ManipulatorBase {

    public Elevator() {
        // TODO: Custom sensor as lower limit switch
        addMotors(new SparkMax(Constants.Wirings.elevatorMotor1, SparkMax.MotorType.kBrushless),
                new SparkMax(Constants.Wirings.elevatorMotor2, SparkMax.MotorType.kBrushless));
        invertSpecificMotors(true, 1);
        setBrakeMode(true);
        setCurrentLimit(Constants.Elevator.amps);
        setPositionMultiplier(Constants.Elevator.positionMultiplier);
        setPositionBounds(Constants.Elevator.positionBoundsMin,
                Constants.Elevator.positionBoundsMax);
        home().schedule();
        setPositionPID(
                new ManipulatorFFMoveCommand(this, 0, 0, Constants.Elevator.posP, Constants.Elevator.posI,
                        Constants.Elevator.posD, ManipulatorFFMoveCommand.FeedForwardType.Elevator,
                        Constants.Elevator.posFFS, Constants.Elevator.posFFV,
                        Constants.Elevator.posFFG, Constants.Elevator.posFFA, Constants.Elevator.maxVelocity,
                        Constants.Elevator.maxAcceleration));
    }

    public void retract() {
        moveToPosition(0);
    }

    /** If falling, send it */
    public void RETRACT() {
        setCurrentLimit(20);
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
                new InstantCommand(() -> _setPosition(0)),
                new InstantCommand(() -> setCurrentLimit(Constants.Elevator.amps)),
                new InstantCommand(() -> fullStop()),
                new InstantCommand(() -> moveToPosition(0)),
                new InstantCommand(() -> getMoveCommand().setEndOnTarget(false)));
    }

    @Override
    public Command test() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> moveToPosition(0)),
                new WaitUntilCommand(() -> isAtPosition()),
                new InstantCommand(() -> moveToPosition(1.5)));
    }

    @Override
    public void moveToPosition(double position) {
        moveToPosition(position, false);
    }
}
