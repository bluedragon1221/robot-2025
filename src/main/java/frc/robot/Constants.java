package frc.robot;

public class Constants {
    public static enum Preset {
        // Height: Don't worry about canrangeOffset in these heights; the subsystem takes care of it
        // Angle: 0rot is horizontal, 0.25rot up, and -0.25rot down because cosine arm mechanics
        Initial         (0,      0.25),
        ScoreL1         (0.1524, 0),
        ScoreL2         (0.1524, 0.0833),
        ScoreL3         (0.3556, 0.0833),
        ScoreL4         (0.6604, 0.1111),
        ExtractAlgaeLow (0.3556, 0),
        ExtractAlgaeHigh(0.6604, 0),
        IntakeCatch     (0.6096, -0.25),
        IntakeGrip      (0.3048, -0.25);

        private double height_meters;
        private double angle;

        private Preset(double h, double a) { height_meters = h; angle = a; }

        public double getHeight() { return height_meters; }
        public double getAngle() { return angle; }
    }

    public static final class ElevatorConstants {
        public static final int leftMotorID = 20;
        public static final int rightMotorID = 21;

        public static final int motorMaxAcceleration = 3;
        public static final int motorCruiseVelocity = 3;

        public static final double motorGearRatio = 7.75;

        public static final double heightTolerance = 0.02;

        public static final double sprocketRadius = 0.0254 * 0.905;

        // public static final double canrangeOffset = 0.16;

        public static final int heightSensorID = 22;
        public static final int bottomSensorDIO = 0;
    }

    public static final class CoralArmPivotConstants {
        public static final int pivotMotorID = 30;
        public static final int pivotEncoderID = 31;

        public static final double pivotMotorAcceleration = 1;
        public static final double pivotMotorCruiseVelocity = 0.5;

        public static final double pivotEncoderOffset = 0.437255859375;

        public static final double pivotMotorCurrentLimit = 40;

        public static final double pivotMotorGearRatio = 37.5;
        public static final double pivotMotorTolerance = 0.1;
    }

    public static final class CoralArmGripperConstants {
        public static final int gripperMotorID = 32;
        public static final double gripperMotorCurrentLimit = 30;
    }

    public static final class AlgaeArmConstants {
        public static final int pivotMotorID = 35;
        public static final int gripperMotorID = 36;

        public static final int pivotMotorAcceleration = 0;
        public static final int pivotMotorCruiseVelocity = 0;

        public static final double pivotMotorGearRatio = 10.0;

        public static final int gripperMotorCurrentLimit = 15;
        public static final double gripperMotorRampRate = 3; // seconds to max voltage
    }

    public static final class ClimberConstants {
        public static final int climberMotorID = 40;

        public static final int climberMotorTolerance = 1;

        public static final int climberMotorCurrentLimit = 60;
    }

    public static final class ElevatorSupersystemConstants {
        public static final int beamBreakSensorDIO = 0;
    }
}
