package frc.robot;

public final class Constants {
    public static class Wirings {
        public static final int swerveModule1DriveMotor = 1;
        public static final int swerveModule1TurningMotor = 2;
        public static final int swerveModule2DriveMotor = 3;
        public static final int swerveModule2TurningMotor = 4;
        public static final int swerveModule3DriveMotor = 5;
        public static final int swerveModule3TurningMotor = 6;
        public static final int swerveModule4DriveMotor = 7;
        public static final int swerveModule4TurningMotor = 8;

        public static final int elevatorMotor1 = 9;
        public static final int elevatorMotor2 = 10;

        public static final int algaeManipulatorMotor = 11;
        public static final int coralManipulatorMotor = 12;
    }

    public static class Swerve {
        public static final double[] drivePID = { 0.5, 0, 0.01 };
        public static final double[] turningPID = { 0.5, 0, 0.01 };
        public static final double[] botTurnPID = { 0.5, 0, 0.1 };
        public static final int driveAmps = 10;
        public static final int turnAmps = 10;

        
        public static final double manipulatorCurrentLimit = 10;
    }
    
    public static class Elevator {        
        public static final double positionMultiplier = 100;
        public static final double positionBoundsMin = 0;
        public static final double positionBoundsMax = 2;
        public static final double posP = 0.1;
        public static final double posI = 0;
        public static final double posD = 0;
        public static final double posTolerance = 0.1;
        public static final double amps = 10;

    }
}
