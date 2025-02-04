package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;

public final class Constants {
    public static final class ElevatorConstants {
        public static final int leftMotorID = 20;
        public static final int rightMotorID = 21;

        public static final int motorMaxAcceleration = 160;
        public static final int motorCruiseVelocity = 80;

        public static final int heightSensorID = 22;
        public static final int bottomSensorGPIO = 0;
    }

    public static final class CoralArmConstants {
        public static final int pivotMotorID = 31;
        public static final int pivotEncoderID = 32;
        public static final int pivotMotorAcceleration = 160;
        public static final int pivotMotorCruiseVelocity = 80;

        public static final Angle pivotMotorTolerance = Degrees.of(1.0);

        public static final int gripperMotorID = 33;
    }

    public static final class AlgaeArmConstants {
        public static final int pivotMotorID = 35;
        public static final int gripperMotorID = 36;
    }

    public static final class ClimberConstants {}

    public static final class ElevatorSupersystemConstants {}
}
