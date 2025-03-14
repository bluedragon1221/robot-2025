package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final double fieldLength   = Units.inchesToMeters(690.876);
    public static final double fieldWidth    = Units.inchesToMeters(317);
    public static final double startingLineX =
        Units.inchesToMeters(299.438); // Measured from the inside of starting line
  
    public static class Reef {
        public static final Translation2d center = new Translation2d(Units.inchesToMeters(176.746),
                Units.inchesToMeters(158.501));
        public static final double faceToZoneLine = Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

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
        public static final ArrayList<Map<ReefHeight, Pose3d>> branchPositions = new ArrayList<>(13);

        static {
            // Initialize branch positions
            for (int face = 0; face < 6; face++) {
                Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
                Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
                for (var level : ReefHeight.values()) {
                    Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
                    double adjustX = Units.inchesToMeters(30.738);
                    double adjustY = Units.inchesToMeters(6.469);

                    fillRight.put(
                        level,
                        new Pose3d(
                            new Translation3d(
                                poseDirection
                                    .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                                    .getX(),
                                poseDirection
                                    .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                                    .getY(),
                                level.height),
                            new Rotation3d(
                                0,
                                Units.degreesToRadians(level.pitch),
                                poseDirection.getRotation().getRadians())));
                    fillLeft.put(
                        level,
                        new Pose3d(
                            new Translation3d(
                                poseDirection
                                    .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                                    .getX(),
                                poseDirection
                                    .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                                    .getY(),
                                level.height),
                            new Rotation3d(
                                0,
                                Units.degreesToRadians(level.pitch),
                                poseDirection.getRotation().getRadians())));
                }
                branchPositions.add(fillLeft);
                branchPositions.add(fillRight);
            }
        }
    }
}
