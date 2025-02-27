package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class Constants {
    public static final class ElevatorConstants {
        public static final int leftMotorID = 20;
        public static final int rightMotorID = 21;

        public static final int motorMaxAcceleration = 48;
        public static final int motorCruiseVelocity = 70;

        public static final double motorGearRatio = 1.0;

        public static final Distance heightTolerance = Inches.of(1);

        public static final Distance sprocketRadius = Inches.of(1);

        public static final int heightSensorID = 22;
        public static final int bottomSensorDIO = 0;
    }

    public static final class CoralArmConstants {
        public static final int pivotMotorID = 31;
        public static final int pivotEncoderID = 32;
        public static final int gripperMotorID = 33;

        public static final int pivotMotorAcceleration = 160;
        public static final int pivotMotorCruiseVelocity = 80;

        public static final Angle pivotMotorTolerance = Degrees.of(1.0);

    }

    public static final class AlgaeArmConstants {
        public static final int pivotMotorID = 35;
        public static final int gripperMotorID = 36;

        public static final int pivotMotorAcceleration = 0;
        public static final int pivotMotorCruiseVelocity = 0;

        public static final double pivotMotorGearRatio = 5.0; // TODO: get actual gear ratio
    }

    public static final class ClimberConstants {
        public static final int climberMotorID = 40;

        public static final int climberMotorTolerance = 1;

        public static final int climberMotorCurrentLimit = 60;
    }

    public static final class ElevatorSupersystemConstants {
    }
}
