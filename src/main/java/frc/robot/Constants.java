package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    public static enum Preset {
        // Height: Don't worry about canrangeOffset in these heights; the subsystem takes care of it
        // Angle: 0 degrees is horizontal, 90 up, and -90 down because cosine arm mechanics
        Initial         (Inches.of(0),  Degrees.of(90)), // I think the angle here is wrong becuase we can't go 90deg up
        ScoreL1         (Inches.of(6),  Degrees.of(0)),
        ScoreL2         (Inches.of(6),  Degrees.of(30)),
        ScoreL3         (Inches.of(14), Degrees.of(30)),
        ScoreL4         (Inches.of(26), Degrees.of(40)),
        ExtractAlgaeLow (Inches.of(14), Degrees.of(0)),
        ExtractAlgaeHigh(Inches.of(26), Degrees.of(0)),
        IntakeCatch     (Inches.of(24), Degrees.of(-90)),
        IntakeGrip      (Inches.of(12), Degrees.of(-90));

        private Distance height;
        private Angle angle;

        private Preset(Distance h, Angle a) { height = h; angle = a; }

        public Distance getHeight() { return height; }
        public Angle getAngle() { return angle; }

        public Preset angleMinus(Angle o) {
            angle = angle.minus(o);
            return this;
        }
    }

    public static final class ElevatorConstants {
        public static final int leftMotorID = 20;
        public static final int rightMotorID = 21;

        public static final int motorMaxAcceleration = 48;
        public static final int motorCruiseVelocity = 70;

        public static final double motorGearRatio = 7.75;

        public static final Distance heightTolerance = Inches.of(1);
        public static final Distance heightToleranceLoose = Inches.of(3);
        public static final Angle angleTolerance = Rotations.of(0.5);

        public static final Distance sprocketRadius = Inches.of(1);

        public static final Distance canrangeOffset = Millimeters.of(128.5875);

        public static final int heightSensorID = 22;
        public static final int bottomSensorDIO = 0;
    }

    public static final class CoralArmConstants {
        public static final int pivotMotorID = 30;
        public static final int pivotEncoderID = 31;
        public static final int gripperMotorID = 32;

        public static final int pivotMotorAcceleration = 50;
        public static final int pivotMotorCruiseVelocity = 17;

        public static final Current pivotMotorCurrentLimit = Amps.of(40);

        public static final double pivotMotorGearRatio = 40.0;
        public static final Angle pivotMotorTolerance = Degrees.of(1.0);

        public static final Current gripperMotorCurrentLimit = Amps.of(30);
    }

    public static final class AlgaeArmConstants {
        public static final int pivotMotorID = 35;
        public static final int gripperMotorID = 36;

        public static final int pivotMotorAcceleration = 0;
        public static final int pivotMotorCruiseVelocity = 0;

        public static final double pivotMotorGearRatio = 10.0;

        public static final Current gripperMotorCurrentLimit = Amps.of(15);
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
