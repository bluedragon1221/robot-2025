//@formatter:off

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.supersystem.Supersystem;

public class AutoRoutines {
    private final AutoFactory factory;

    public AutoRoutines(AutoFactory factory) {
        this.factory = factory;
    }

    public AutoRoutine Center1l4() {
        final AutoRoutine routine = factory.newRoutine("Center 1L4");
        final AutoTrajectory path = routine.trajectory("center_1l4", 0);
        final AutoTrajectory driveback = routine.trajectory("center_1l4", 1);

        routine.active().onTrue(Commands.sequence(
            path.resetOdometry(),
            path.cmd())
        );

        path.atTime("Storage Position").onTrue(Supersystem.storage.coral());
        path.atTime("Prepare L4").onTrue(Supersystem.coral.prepareL4());

        path.atTime("Score L4")
            .onTrue(Commands.waitUntil(Supersystem.coral.canScoreL4).andThen(Supersystem.coral.scoreL4()));

        path.recentlyDone().and(Supersystem.coral.hasScoredL4).onTrue(driveback.cmd());
        driveback.recentlyDone().onTrue(Supersystem.storage.empty());

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
            drive_to_1l4.cmd())
        );

        drive_to_1l4.atTime("Storage Position").onTrue(Supersystem.storage.empty());
        drive_to_1l4.atTime("Prepare L4").onTrue(Supersystem.coral.prepareL4());
        drive_to_1l4.atTime("Score L4")
                        .onTrue(Commands.waitUntil(Supersystem.coral.canScoreL4).andThen(Supersystem.coral.scoreL4()));

        drive_to_1l4.recentlyDone().and(Supersystem.coral.hasScoredL4).onTrue(drive_to_1hps.cmd());

        drive_to_1hps.atTime("Prepare Intake").onTrue(Supersystem.intake.prepare());
        drive_to_1hps.recentlyDone().onTrue(
            Commands.waitSeconds(1.5) // Time for HP to place coral
                .andThen(Supersystem.intake.load()
                    .until(Supersystem.intake.hasIntaked)
                    .withTimeout(5))
                .andThen(drive_to_2l4.spawnCmd()));

        drive_to_2l4.atTime("Prepare L4 2").and(Supersystem.hasCoral).onTrue(Supersystem.coral.prepareL4());
        drive_to_2l4.atTime("Score L4 2").and(Supersystem.hasCoral)
            .onTrue(Commands.waitUntil(Supersystem.coral.canScoreL4).andThen(Supersystem.coral.scoreL4()));

        drive_to_2l4.recentlyDone().and(Supersystem.coral.hasScoredL4).onTrue(driveback.cmd());

        driveback.atTime("Storage Position 2").onTrue(Supersystem.storage.coral());
        driveback.recentlyDone().onTrue(Supersystem.storage.coral());

        return routine;
    }

    public AutoRoutine DriveForward() {
        final AutoRoutine routine = factory.newRoutine("Drive Forward");
        final AutoTrajectory path = routine.trajectory("drive_forward");

        routine.active().onTrue(Commands.sequence(
            path.resetOdometry(),
            path.cmd())
        );

        path.atTime("Storage Position").onTrue(Supersystem.storage.empty());

        return routine;
    }
}
