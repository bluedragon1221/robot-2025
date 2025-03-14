package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine CenterCage3l4() {
        final AutoRoutine routine = m_factory.newRoutine("Center Cage 2L4");
        final AutoTrajectory path = routine.trajectory("centercage_3l4");

        routine.active().onTrue(
            path.resetOdometry().andThen(path.spawnCmd())
        );

        return routine;
    }

    public AutoRoutine BottomCenterCage3l4() {
        final AutoRoutine routine = m_factory.newRoutine("Bottom Center Cage 2L4");
        final AutoTrajectory path = routine.trajectory("bottom_centercage_3l4");

        routine.active().onTrue(
            path.resetOdometry().andThen(path.spawnCmd())
        );

        return routine;
    }

    public AutoRoutine Center1l4() {
        final AutoRoutine routine = m_factory.newRoutine("Center 1L4");
        final AutoTrajectory path = routine.trajectory("center_1l4");

        routine.active().onTrue(
            path.resetOdometry().andThen(path.spawnCmd())
        );

        return routine;
    }

    public AutoRoutine DriveForward() {
        final AutoRoutine routine = m_factory.newRoutine("Drive Forward");
        final AutoTrajectory path = routine.trajectory("drive_forward");

        routine.active().onTrue(
            path.resetOdometry().andThen(path.spawnCmd())
        );

        return routine;
    }
}
