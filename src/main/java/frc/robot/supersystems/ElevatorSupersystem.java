package frc.robot.supersystems;

import static frc.robot.Constants.ElevatorSupersystemConstants.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Preset;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;

public class ElevatorSupersystem extends SubsystemBase {
    private static ElevatorSupersystem instance;

    private static Elevator elevator;
    private static CoralArm coral_arm;

    private static DigitalInput beamBreakSensor;

    private ElevatorSupersystem() {
        elevator = Elevator.getInstance();
        coral_arm = CoralArm.getInstance();

        beamBreakSensor = new DigitalInput(beamBreakSensorDIO);
    }

    public static synchronized ElevatorSupersystem getInstance() {
        if (instance == null) {
            instance = new ElevatorSupersystem();
        }

        return instance; 
    }

    public boolean hasCoral() {
        return beamBreakSensor.get();
    }

    // Intake
    public Command intakeSetupIntake() {
        return run(() -> {
            elevator.setHeightFromPreset(Preset.IntakeCatch);
            coral_arm.setPivotAngleFromPreset(Preset.IntakeCatch);
        });
    }
    public Command intakeLoadIntake() {
        if (elevator.isAtHeightFromPreset(Preset.IntakeCatch, Inches.of(3)) &&
                coral_arm.isAtAngleFromPreset(Preset.IntakeCatch, Degrees.of(2))) {
            return run(() -> {
                elevator.setHeightFromPreset(Preset.IntakeGrip); // ideally this would run slower than usual
                
                coral_arm.setGripperVoltage(Volts.of(2))
                    .until(this::hasCoral)
                    .andThen(() -> coral_arm.setGripperVoltage(Volts.of(0)));                
            });
        } else {
            return Commands.none();
        }
    }

    // Score Coral
    public static enum CoralLayer {
        L1, L2, L3, L4;
    
        public Preset toPreset() {
            return switch (this) {
                case L1 -> Preset.ScoreL1;
                case L2 -> Preset.ScoreL2;
                case L3 -> Preset.ScoreL3;
                case L4 -> Preset.ScoreL4;
            };
        }
    };
    public Command coralPrepareElevator(CoralLayer selected_layer) {
        return run(() -> {
            elevator.setHeightFromPreset(selected_layer.toPreset());
        }).until(elevator.isAtHeightFromPreset(selected_layer.toPreset()));
    }
    public Command coralPrepareArm(CoralLayer selected_layer) {
        return run(() -> {
            coral_arm.setPivotAngleFromPreset(selected_layer.toPreset());
        }).until(() -> coral_arm.isAtAngleFromPreset(selected_layer.toPreset()));
    }
    public Command coralScoreCoral(CoralLayer selected_layer) {
        if (selected_layer == CoralLayer.L1) {
            // it's at 90deg, driver drives forward while we spin gripper motors negative
            return coral_arm.setGripperVoltage(Volts.of(-3));
        } else if (selected_layer == CoralLayer.L2) {
            // it's at 60deg. needs to rotate down, then driver drives away
            return run(() -> coral_arm.setPivotAngleFromPreset(
                Preset.ScoreL2.angleMinus(Degrees.of(10))) // rotate down 10deg
            );
        } else if (selected_layer == CoralLayer.L3) {
            return run(() -> coral_arm.setPivotAngleFromPreset(
                Preset.ScoreL3.angleMinus(Degrees.of(10)))
            );
        } else if (selected_layer == CoralLayer.L4) {
            return run(() -> coral_arm.setPivotAngleFromPreset(
                Preset.ScoreL4.angleMinus(Degrees.of(10)))
            );
        } else {
            return Commands.none();
        }
    }

    // Extract Algae
    public static enum AlgaeExtractionLayer {
        High, Low;
    
        public Preset toPreset() {
            return switch (this) {
                case High -> Preset.ExtractAlgaeLow;
                case Low  -> Preset.ExtractAlgaeHigh;
            };
        }
    };
    public Command algaeExtractionPrepareElevator(AlgaeExtractionLayer selected_layer) {
        return run(() -> elevator.setHeightFromPreset(selected_layer.toPreset()))
            .until(elevator.isAtHeightFromPreset(selected_layer.toPreset()));
    }
    public Command algaeExtractionPrepareArm() {
        return run(() -> coral_arm.setPivotAngleFromPreset(Preset.ExtractAlgaeLow))
            .until(() -> coral_arm.isAtAngleFromPreset(Preset.ExtractAlgaeLow)); // 90deg for both of them, no need to pass it in
    }
    public Command algaeExtractionextractAlgae() {
        return coral_arm.setGripperVoltage(Volts.of(-3));
    }
}
