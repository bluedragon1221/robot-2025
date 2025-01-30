package frc.robot;

public final class Constants {
    public static final class AlgaeArmConstants {
        public static final int pivotMotorID = 0;
        public static final int gripperMotorID = 0;
    }

    public static final class CoralArmConstants {
        public static final int pivotMotorID = 0;
        public static final int pivotMotorAcceleration = 160;
        public static final int pivotMotorCruiseVelocity = 80;

        public static final int pivotMotorTolerance = 1; // IN DEGREES

        public static final int pivotEncoderID = 0;
        public static final int gripperMotorID = 0;
    }

    public static final class ElevatorConstants {
        public static final int leftMotorID = 0;
        public static final int rightMotorID = 0;

        public static final int motorAcceleration = 160;
        public static final int motorCruiseVelocity = 80;

        public static final int heightSensorID = 0;
        public static final int bottomSensorGPIO = 0;
    }
}
