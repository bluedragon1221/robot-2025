package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;

/**
 * Contains various field dimensions and useful reference points. All units are
 * in meters and poses have a blue alliance
 * origin.
 */
public class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);
    public static final double startingLineX = Units.inchesToMeters(299.438); // Measured from the inside of starting line

    public enum ReefHeight {
        L4(Units.inchesToMeters(54), 44),
        L3(Units.inchesToMeters(48), 14),
        L2(Units.inchesToMeters(42), -14),
        L1(Units.inchesToMeters(42), -56);

        public final double height;
        public final double pitch;

        ReefHeight(double height, double pitch) {
            this.height = height;
            this.pitch = pitch; // in degrees
        }
    }

    public static class Reef {
         // Side of the reef to the inside of the reef zone line
        public static final Translation2d center = new Translation2d(
            Units.inchesToMeters(176.746),
            Units.inchesToMeters(158.501)
        );

        public static final Pose2d[] centerFaces = new Pose2d[] {
                new Pose2d(
                        Units.inchesToMeters(144.003),
                        Units.inchesToMeters(158.500),
                        Rotation2d.fromDegrees(180)),
                new Pose2d(
                        Units.inchesToMeters(160.373),
                        Units.inchesToMeters(186.857),
                        Rotation2d.fromDegrees(120)),
                new Pose2d(
                        Units.inchesToMeters(193.116),
                        Units.inchesToMeters(186.858),
                        Rotation2d.fromDegrees(60)),
                new Pose2d(
                        Units.inchesToMeters(209.489),
                        Units.inchesToMeters(158.502),
                        Rotation2d.fromDegrees(0)),
                new Pose2d(
                        Units.inchesToMeters(193.118),
                        Units.inchesToMeters(130.145),
                        Rotation2d.fromDegrees(-60)),
                new Pose2d(
                        Units.inchesToMeters(160.375),
                        Units.inchesToMeters(130.144),
                        Rotation2d.fromDegrees(-120))
        }; // Starting facing the driver station in clockwise order


        public static ArrayList<Pose2d> lefts = new ArrayList<>();
        public static ArrayList<Pose2d> rights = new ArrayList<>();

        static {
            for (int face = 0; face < 6; face++) {
                Pose2d centerWithAngle = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
                double adjustX = Units.inchesToMeters(30.738+19); // set 19in back for aligning
                double adjustY = Units.inchesToMeters(6.469);

                lefts.add(new Pose2d(
                    new Translation2d(
                        centerWithAngle
                            .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                            .getX(),
                        centerWithAngle
                             .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                            .getY()
                    ),
                    new Rotation2d(centerWithAngle.getRotation().getRadians())
                ));

                rights.add(new Pose2d(
                    new Translation2d(
                        centerWithAngle
                            .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                            .getX(),
                        centerWithAngle
                            .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                            .getY()
                    ),
                    new Rotation2d(centerWithAngle.getRotation().getRadians())
                ));
            }
        }
    }
}
