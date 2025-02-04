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

    public static final int algaeClawMotor = 11;
    public static final int algaeArmMotor = 12;
    public static final int coralManipulatorMotor = 13;
  }

  public static class Swerve {
    public static final int driveAmps = 20;
    public static final int turnAmps = 20;

    public static final double module1Offset = 0.4547673;
    public static final double module2Offset = 0.3288317;
    public static final double module3Offset = 0.2699317;
    public static final double module4Offset = 0.5163250;

    public static final String module1Name = "Front right";
    public static final String module2Name = "Front left";
    public static final String module3Name = "Back left";
    public static final String module4Name = "Back right";
    public static final double sideLength = 29.75;

    public static class Movement {
      public static final double maxSpeed = 1;
      public static final double maxAccel = 1;
      public static final double maxRotationSpeed = 1;
      public static final double maxRotationAccel = 1;

      public static final double driveGearRatio = 1 / (10 * Math.PI * 15 / 50);

      public static class Drive {
        public static final double P = 0.1;
        public static final double I = 0;
        public static final double D = 0;
        public static final double S = 0;
        public static final double V = 0;
        public static final double A = 0;
        public static final double[] PIDF = { P, I, D, S, V, A };
      }

      public static class Turn {
        public static final double P = 0.1;
        public static final double I = 0;
        public static final double D = 0;
        public static final double[] PID = { P, I, D };
      }

      public static class BotTurn {
        public static final double P = 0.1;
        public static final double I = 0;
        public static final double D = 0;
        public static final double S = 0;
        public static final double V = 0;
        public static final double A = 0;
        public static final double[] PIDF = { P, I, D, S, V, A };
      }
    }
  }

  public static class Elevator {
    public static final int amps = 20;
    public static final double positionMultiplier = 100; // TODO find elevator position multiplier to be in meters
    public static final double positionBoundsMin = 0; // TODO find elevator position bounds
    public static final double positionBoundsMax = 2;

    public static final double P = 0.1; // TODO find elevator PID values
    public static final double I = 0;
    public static final double D = 0;
    public static final double S = 0; // TODO find elevator FF values
    public static final double V = 0;
    public static final double G = 0;
    public static final double A = 0;
    public static final double[] PIDF = { P, I, D, S, V, G, A };

    public static final double maxVelocity = 0.25; // TODO find elevator max velocity
    public static final double maxAcceleration = 0.25;
  }

  public static class AlgaeClaw {
    public static final int currentLimit = 10;
  }

  public static class AlgaeArm {
    public static final int currentLimit = 10;
    public static final double positionMultiplier = 100; // TODO find arm position multiplier to be in radians
    public static final double positionBoundsMin = Math.PI / 2;
    public static final double positionBoundsMax = -Math.PI / 2;

    public static final double P = 0.1; // TODO find arm PID values
    public static final double I = 0;
    public static final double D = 0;
    public static final double S = 0; // TODO find arm FF values
    public static final double V = 0;
    public static final double G = 0;
    public static final double A = 0;
    public static final double[] PIDF = { P, I, D, S, V, G, A };

    public static final double maxVelocity = 0.25; // TODO find arm max velocity
    public static final double maxAcceleration = 0.25;
  }

  public static class CoralManipulator {
    public static final int currentLimit = 10;
  }
}
