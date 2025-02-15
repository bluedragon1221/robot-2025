package frc.robot.supersystems;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralArm.PivotPreset;
import frc.robot.subsystems.Elevator.HeightPreset;

public class ElevatorSupersystem extends SubsystemBase {
    private static Elevator elevator;
    private static CoralArm coral_arm;

    public ElevatorSupersystem() {
        elevator = Elevator.getInstance();
        coral_arm = CoralArm.getInstance();
    }

    public void seqFullIntake() {
        elevator.setHeightFromPreset(HeightPreset.Intake);
        coral_arm.setPivotAngleFromPreset(PivotPreset.Down);
        
        Time timeout = Milliseconds.of(100);
        coral_arm.intake(timeout);

    }
}
