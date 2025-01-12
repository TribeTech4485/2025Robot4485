package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorBase;

public class Elevator extends ManipulatorBase {

    public Elevator(CANSparkMax motor1, CANSparkMax motor2) {
        addMotors(motor1, motor2);
        invertSpecificMotors(true, 1);
        setBrakeMode(true);
        // setCurrentLimit(10); // is this enough?
        setPositionPID(Constants.Elevator.posP, Constants.Elevator.posI, Constants.Elevator.posD,
                Constants.Elevator.posTolerance); // TODO find elevator PID values
        setPositionMultiplier(Constants.Elevator.positionMultiplier); // TODO find elevator position multiplier to be in meters
        setPositionBounds(Constants.Elevator.positionBoundsMin, Constants.Elevator.positionBoundsMax); // TODO find elevator position bounds
        home().schedule();
    }

    public void retract() {
        moveToPosition(0);
    }

    /** If falling, send it */
    public void RETRACT() {
        setCurrentLimit(20);
        setMaxPower(1);
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
                new InstantCommand(() -> setCurrentLimit(20)),
                new InstantCommand(() -> setPower(0.1)),
                new WaitUntilCommand(customSensor),
                new InstantCommand(() -> _setPosition(0)),
                new InstantCommand(() -> setCurrentLimit(10)),
                new InstantCommand(() -> stop()));
    }

    @Override
    public Command test() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> moveToPosition(0)),
                new WaitUntilCommand(() -> isAtPosition()),
                new InstantCommand(() -> moveToPosition(1.5)));
    }
}
