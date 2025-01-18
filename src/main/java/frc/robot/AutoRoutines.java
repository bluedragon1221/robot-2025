package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine testAuto() {
        final AutoRoutine routine = m_factory.newRoutine("test_auto");
        final AutoTrajectory simplePath = routine.trajectory("bottom_2l4");

        routine.active().onTrue(
            simplePath.resetOdometry().andThen(simplePath.cmd())
        );

        return routine;
    }
}
