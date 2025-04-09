package frc.robot;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.supersystems.ElevatorSupersystem;
import frc.robot.util.AllianceFlipUtil;

public class AutoRoutines {
        private final AutoFactory factory;
        private final CommandSwerveDrivetrain drivetrain;

        public AutoRoutines(AutoFactory factory, CommandSwerveDrivetrain drivetrain) {
                this.factory = factory;
                this.drivetrain = drivetrain;
        }

        public AutoRoutine Center1l4() {
                final AutoRoutine routine = factory.newRoutine("Center 1L4");
                final AutoTrajectory path = routine.trajectory("center_1l4", 0);
                final AutoTrajectory driveback = routine.trajectory("center_1l4", 1);

                routine.active().onTrue(Commands.sequence(
                                path.resetOdometry(),
                                path.cmd()));

                path.atTime("Storage Position").onTrue(ElevatorSupersystem.storage.coral());
                path.atTime("Prepare L4").onTrue(ElevatorSupersystem.coral.prepareL4());

                path.atTime("Score L4")
                                .onTrue(Commands.waitUntil(ElevatorSupersystem.coral.canScoreL4).andThen(ElevatorSupersystem.coral.scoreL4()));

                path.recentlyDone().and(ElevatorSupersystem.coral.hasScoredL4).onTrue(driveback.cmd());
                driveback.recentlyDone().onTrue(ElevatorSupersystem.storage.empty());

                return routine;
        }

        public AutoRoutine BlueCenterCage2l4() {
                final AutoRoutine routine = factory.newRoutine("Blue Center Cage 2L4");
                final AutoTrajectory drive_to_1l4 = routine.trajectory("blue_centercage_2l4", 0);
                final AutoTrajectory drive_to_1hps = routine.trajectory("blue_centercage_2l4", 1);
                final AutoTrajectory drive_to_2l4 = routine.trajectory("blue_centercage_2l4", 2);
                final AutoTrajectory driveback = routine.trajectory("blue_centercage_2l4", 3);

                routine.active().onTrue(Commands.sequence(
                                drive_to_1l4.resetOdometry(),
                                drive_to_1l4.cmd()));

                drive_to_1l4.atTime("Storage Position").onTrue(ElevatorSupersystem.storage.empty());
                drive_to_1l4.atTime("Prepare L4").onTrue(ElevatorSupersystem.coral.prepareL4());
                drive_to_1l4.atTime("Score L4")
                                .onTrue(Commands.waitUntil(ElevatorSupersystem.coral.canScoreL4).andThen(ElevatorSupersystem.coral.scoreL4()));

                drive_to_1l4.recentlyDone().and(ElevatorSupersystem.coral.hasScoredL4).onTrue(drive_to_1hps.cmd());

                drive_to_1hps.atTime("Prepare Intake").onTrue(ElevatorSupersystem.intake.prepare());
                drive_to_1hps.recentlyDone().onTrue(
                                Commands.waitSeconds(1.5) // Time for HP to place coral
                                                .andThen(ElevatorSupersystem.intake.load()
                                                                .until(ElevatorSupersystem.intake.hasIntaked)
                                                                .withTimeout(5))
                                                .andThen(drive_to_2l4.spawnCmd()));

                drive_to_2l4.atTime("Prepare L4 2").and(ElevatorSupersystem.hasCoral).onTrue(ElevatorSupersystem.coral.prepareL4());
                drive_to_2l4.atTime("Score L4 2").and(ElevatorSupersystem.hasCoral)
                                .onTrue(Commands.waitUntil(ElevatorSupersystem.coral.canScoreL4).andThen(ElevatorSupersystem.coral.scoreL4()));

                drive_to_2l4.recentlyDone().and(ElevatorSupersystem.coral.hasScoredL4).onTrue(driveback.cmd());

                driveback.atTime("Storage Position 2").onTrue(ElevatorSupersystem.storage.coral());
                driveback.recentlyDone().onTrue(ElevatorSupersystem.storage.coral());

                return routine;
        }

        public AutoRoutine DriveForward() {
                final AutoRoutine routine = factory.newRoutine("Drive Forward");
                final AutoTrajectory path = routine.trajectory("drive_forward");

                routine.active().onTrue(Commands.sequence(
                                path.resetOdometry(),
                                path.cmd()));

                path.atTime("Storage Position").onTrue(ElevatorSupersystem.storage.empty());

                return routine;
        }
}
