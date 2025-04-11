//@formatter:off

package frc.robot.supersystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coralarmgripper.CoralArmGripperPreset;
import frc.robot.subsystems.coralarmgripper.CoralArmGripperSubsystem;
import frc.robot.subsystems.coralarmpivot.CoralArmPivotPreset;
import frc.robot.subsystems.coralarmpivot.CoralArmPivotSubsystem;
import frc.robot.subsystems.elevator.ElevatorPreset;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class Supersystem {
    private static final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    private static final CoralArmGripperSubsystem coral_gripper = CoralArmGripperSubsystem.getInstance();
    private static final CoralArmPivotSubsystem coral_pivot = CoralArmPivotSubsystem.getInstance();

    public static final Trigger pivotSafe = coral_pivot.isBetweenAngles(CoralArmPivotPreset.minSafe, CoralArmPivotPreset.maxSafe);
    public static final Trigger hasCoral = coral_gripper.hasCoral;

    public static final class storage {
        public static Command empty() {
            return coral();
        }
    
        public static Command coral() {
            // case 1: coming from robot start
                // set elevator (should already be initial)
                // parallel set arm
            // case 2: coming from l4 or barge
                // set arm to storage
                // wait until below highest safe pivot, then move elevator
            // case 3: coming from postIntake
                // set arm to storage
                // wait until above lowest safe pivot (0.01), then move elevator
            // or generally:
                // set arm to storage
                // wait until between highest and lowest safe pivot, then move elevator

            return Commands.parallel(
                coral_pivot.setAngle(CoralArmPivotPreset.storage),
                Commands.waitUntil(pivotSafe)
                    .andThen(elevator.setHeight(ElevatorPreset.storage))
            );
        }

        public static Command algae() {
            return empty()
                .alongWith(coral_gripper.setVoltage(CoralArmGripperPreset.holdAlgae));
        }
    }
    
    public static final class intake {
        public static Command prepare() {
            // case 1: coming from storage
                // set angle to minSafe
                // parallel set height to intake catch
                // wait until elevator is at safe height, then set angle to intakeCatch
            // case 2: coming from l4 or barge
                // set angle to intakeCatch (safe because we're already above intakeCatch height)
                // parallel set height to intakeCatch
            // case 3: broken state, angle beyond maxSafe
                // set elevator to maxSafe, then case 1

            return Commands.either(
                // case 1, 3: coming from storage (deals with any angle)
                Commands.parallel(
                    coral_pivot.setAngle(CoralArmPivotPreset.minSafe)
                        .until(elevator.isAtHeight(ElevatorPreset.intakeCatch))
                        .andThen(coral_pivot.setAngle(CoralArmPivotPreset.intake)),
                    Commands.waitUntil(pivotSafe)
                        .andThen(elevator.setHeight(ElevatorPreset.intakeCatch))
                ),
                // case 2: coming from l4
                Commands.parallel(
                    coral_pivot.setAngle(CoralArmPivotPreset.intake),
                    elevator.setHeight(ElevatorPreset.intakeCatch)
                ),
                elevator.isAtHeight(ElevatorPreset.storage)
            );
        }

        public static final Trigger isPrepared = elevator.isAtHeight(ElevatorPreset.intakeCatch)
            .and(coral_pivot.isAtAngle(CoralArmPivotPreset.intake));

        public static Command load() {
            // only run if already prepared
            // set gripper voltage and set intakeGrip height until has coral
            // set gripper voltage to holdCoral and intakePost height

            return Commands.sequence(
                Commands.parallel(
                    coral_gripper.setVoltage(CoralArmGripperPreset.intakeCoral),
                    elevator.setHeight(ElevatorPreset.intakeGrip)
                ).until(hasCoral).withTimeout(5),
                Commands.parallel(
                    coral_gripper.setVoltage(CoralArmGripperPreset.holdCoral),
                    elevator.setHeight(ElevatorPreset.intakePost)
                )
            ).onlyIf(isPrepared);
        }

        public static final Trigger hasIntaked = elevator.isAtHeight(ElevatorPreset.intakePost)
            .and(coral_pivot.isAtAngle(CoralArmPivotPreset.intake))
            .and(hasCoral);
    }

    public static final class coral {
        // prepare:
            // go to the correct angle
            // wait until above min safe angle, then set angle to correct height

        private static Command prepare(double angle, double height) {
            return Commands.parallel(
                coral_pivot.setAngle(angle),
                Commands.waitUntil(pivotSafe)
                    .andThen(elevator.setHeight(height))
            );
        }

        public static Command prepareL4() {
            return prepare(CoralArmPivotPreset.prepareL4, ElevatorPreset.l4);
        }

        public static Command prepareL3() {
            return prepare(CoralArmPivotPreset.prepareL3, ElevatorPreset.l3);
        }

        public static Command prepareL2() {
            return prepare(CoralArmPivotPreset.prepareL2, ElevatorPreset.l2);
        }

        public static Command prepareL1() {
            return prepare(CoralArmPivotPreset.prepareL1, ElevatorPreset.l1);
        }

        // score:
            // set pivot to scoreL4
            // spin grippers at -0.5 for 1 sec

        public static final Trigger canScoreL4 = elevator.isAtHeight(ElevatorPreset.l4)
            .and(coral_pivot.isAtAngle(CoralArmPivotPreset.prepareL4));
        public static final Trigger hasScoredL4 = elevator.isAtHeight(ElevatorPreset.l4)
            .and(coral_pivot.isAtAngle(CoralArmPivotPreset.scoreL4));
        public static Command scoreL4() {
            return Commands.parallel(
                coral_pivot.setAngle(CoralArmPivotPreset.scoreL4),
                Commands.waitUntil(coral_pivot.isAtAngle(CoralArmPivotPreset.scoreL4))
                    .andThen(coral_gripper.setVoltage(CoralArmGripperPreset.releaseCoralL1))
            ).onlyIf(canScoreL4);
        }

        public static final Trigger canScoreL3 = elevator.isAtHeight(ElevatorPreset.l3)
            .and(coral_pivot.isAtAngle(CoralArmPivotPreset.scoreL3));
        public static Command scoreL3() {
            return Commands.parallel(
                coral_pivot.setAngle(CoralArmPivotPreset.scoreL3),
                coral_gripper.setVoltage(CoralArmGripperPreset.releaseCoralL3)
            ).onlyIf(canScoreL3);
        }

        public static final Trigger canScoreL2 = elevator.isAtHeight(ElevatorPreset.l2)
            .and(coral_pivot.isAtAngle(CoralArmPivotPreset.scoreL2));
        public static Command scoreL2() {
            return Commands.parallel(
                coral_pivot.setAngle(CoralArmPivotPreset.scoreL2),
                coral_gripper.setVoltage(CoralArmGripperPreset.releaseCoralL2)
            ).onlyIf(canScoreL2);
        }

        public static final Trigger canScoreL1 = elevator.isAtHeight(ElevatorPreset.l1)
            .and(coral_pivot.isAtAngle(0));

        public static Command scoreL1() {
            return coral_gripper.setVoltage(CoralArmGripperPreset.releaseCoralL1).onlyIf(canScoreL1);
        }
    }

    public static final class extraction {
        public static Command prepareLow() {
            return Commands.parallel(
                coral_pivot.setAngle(CoralArmPivotPreset.extractAlgae),
                Commands.waitUntil(pivotSafe).andThen(elevator.setHeight(ElevatorPreset.extractAlgaeLow))
            );
        }

        public static Command prepareHigh() {
            return Commands.parallel(
                coral_pivot.setAngle(CoralArmPivotPreset.extractAlgae),
                Commands.waitUntil(pivotSafe).andThen(elevator.setHeight(ElevatorPreset.extractAlgaeHigh))
            );
        }

        public static final Trigger canExtractLow = elevator.isAtHeight(ElevatorPreset.extractAlgaeLow)
            .and(coral_pivot.isAtAngle(CoralArmPivotPreset.extractAlgae));
        
        public static final Trigger canExtractHigh = elevator.isAtHeight(ElevatorPreset.extractAlgaeHigh)
            .and(coral_pivot.isAtAngle(CoralArmPivotPreset.extractAlgae));
        
        public static Command extract() {
            return coral_gripper.setVoltage(CoralArmGripperPreset.extractAlgae)
                .until(coral_gripper.hasCoral)
                .withTimeout(3);
        }
    }

    public static final class processor {
        public static Command prepare() {
            return Commands.parallel(
                // This should continue over gripper voltage from previous state
                    // case 1: coming from storage.algae
                    // case 2: coming from extraction
                coral_pivot.setAngle(CoralArmPivotPreset.scoreProcessor),
                Commands.waitUntil(pivotSafe)
                    .andThen(elevator.setHeight(ElevatorPreset.scoreProcessor))
            );
        }

        public static Command score() {
            return coral_gripper
                .setVoltage(CoralArmGripperPreset.scoreAlgaeProcessor)
                .withTimeout(1)
                .andThen(coral_gripper.setVoltage(CoralArmGripperPreset.zero));
        }
    }

    public static final class barge {
        public static Command prepare() {
            return Commands.parallel(
                coral_pivot.setAngle(CoralArmPivotPreset.maxSafe)
                    .until(elevator.isAtHeight(ElevatorPreset.scoreBarge))
                    .andThen(coral_pivot.setAngle(CoralArmPivotPreset.scoreBarge)),
                Commands.waitUntil(pivotSafe)
                    .andThen(elevator.setHeight(ElevatorPreset.scoreBarge))
            );
        }

        public static Command score() {
            return coral_gripper.setVoltage(CoralArmGripperPreset.scoreAlgaeBarge)
                .withTimeout(1)
                .andThen(coral_gripper.setVoltage(CoralArmGripperPreset.zero));
        }
    }
}
