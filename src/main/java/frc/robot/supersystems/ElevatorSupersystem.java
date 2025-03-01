package frc.robot.supersystems;

import static frc.robot.Constants.ElevatorSupersystemConstants.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Preset;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;

public class ElevatorSupersystem extends SubsystemBase {
    private static Elevator elevator;
    private static CoralArm coral_arm;

    private static DigitalInput beamBreakSensor;

    public ElevatorSupersystem() {
        elevator = Elevator.getInstance();
        coral_arm = CoralArm.getInstance();

        beamBreakSensor = new DigitalInput(beamBreakSensorDIO);
    }

    public boolean hasCoral() {
        return beamBreakSensor.get();
    }

    public Command setupIntake() {
        return run(() -> {
            elevator.setHeightFromPreset(Preset.IntakeCatch);
            coral_arm.setPivotAngleFromPreset(Preset.IntakeCatch);
        });
    }

    public Command loadIntake() {
        if (elevator.isAtHeightFromPreset(Preset.IntakeCatch, Inches.of(3)) &&
                coral_arm.isAtAngleFromPreset(Preset.IntakeCatch, Degrees.of(2))) {
            return run(() -> {
                Commands.parallel(
                    elevator.setHeightFromPreset(Preset.IntakeGrip),
                    coral_arm.setGripperVoltage(Volts.of(2))
                ).until(this::hasCoral)
                .andThen(() -> coral_arm.setGripperVoltage(Volts.of(0)));                
            });
        } else {
            return Commands.none();
        }
    }

    public Command scoreL4() {
        // assume state is ScoreL4 and aligned with reef
        return run(() -> {
            coral_arm.setPivotAngleFromPreset(Preset.ScoreL4.angleMinus(Degrees.of(10)))
                .withTimeout(Milliseconds.of(400));
        });
        // Now driver can drive backward to finish scoring
    }

    public Command scoreL1() {
        return Commands.none();
    }
}
