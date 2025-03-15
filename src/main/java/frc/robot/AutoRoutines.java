package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.supersystems.ElevatorSupersystem;

public class AutoRoutines {
    private final ElevatorSupersystem supersystem = ElevatorSupersystem.getInstance();
    
    private final AutoFactory factory;

    public AutoRoutines(AutoFactory factory) {
        this.factory = factory;
    }

    public AutoRoutine CenterCage3l4() {
        final AutoRoutine routine = factory.newRoutine("Center Cage 2L4");
        final AutoTrajectory path = routine.trajectory("centercage_3l4");

        routine.active().onTrue(
            path.resetOdometry().andThen(path.spawnCmd())
        );

        return routine;
    }

    public AutoRoutine BottomCenterCage3l4() {
        final AutoRoutine routine = factory.newRoutine("Bottom Center Cage 2L4");
        final AutoTrajectory path = routine.trajectory("bottom_centercage_3l4");

        routine.active().onTrue(
            path.resetOdometry().andThen(path.spawnCmd())
        );

        return routine;
    }

    public AutoRoutine Center1l4() {
        final AutoRoutine routine = factory.newRoutine("Center 1L4");
        final AutoTrajectory path = routine.trajectory("center_1l4");

        routine.active().onTrue(Commands.parallel(
            path.resetOdometry(),
            path.cmd()
        ));

        path.atTime("Storage Position").onTrue(supersystem.storagePosition());
        path.atTime("Prepare L4").onTrue(supersystem.coralPrepareL4());
        path.atTime("Score L4").onTrue(supersystem.coralScoreL4());

        return routine;
    }

    public AutoRoutine DriveForward() {
        final AutoRoutine routine = factory.newRoutine("Drive Forward");
        final AutoTrajectory path = routine.trajectory("drive_forward");

        routine.active().onTrue(Commands.sequence(
            path.resetOdometry(),
            path.cmd()
        ));

        path.atTime("Storage Position").onTrue(supersystem.storagePosition());

        return routine;
    }
}
