package frc.robot.supersystems;

import static frc.robot.Constants.ElevatorSupersystemConstants.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Preset;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;

public class ElevatorSupersystem extends SubsystemBase {
    private static ElevatorSupersystem instance;

    private static final Elevator elevator = Elevator.getInstance();
    private static final CoralArm coral_arm = CoralArm.getInstance();

    private static DigitalInput beamBreakSensor;

    private ElevatorSupersystem() {
        beamBreakSensor = new DigitalInput(beamBreakSensorDIO);
    }

    public static synchronized ElevatorSupersystem getInstance() {
        if (instance == null) {
            instance = new ElevatorSupersystem();
        }

        return instance; 
    }

    public Trigger hasCoral() {
        return new Trigger(() -> beamBreakSensor.get());
    }

    // Intake
    public Command intakeSetupIntake() {
        return Commands.parallel(
            elevator.setHeight(Preset.IntakeCatch.getHeight()),
            coral_arm.setAngle(Preset.IntakeCatch.getAngle())
        );
    }
    public Command intakeLoadIntake() {
        if (elevator.getHeight() >= Preset.IntakeCatch.getHeight()) {
            return Commands.parallel(
                elevator.setHeight(Preset.IntakeGrip.getHeight()),
                coral_arm.setGripperVoltage(2)
                    .until(hasCoral())
                    .andThen(coral_arm.setGripperVoltage(0))
            );
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
        return elevator.setHeight(selected_layer.toPreset().getHeight());
    }

    public Command coralPrepareArm(CoralLayer selected_layer) {
        return coral_arm.setAngle(selected_layer.toPreset().getAngle());
    }

    public Command coralScoreCoral(CoralLayer selected_layer) {
        if (selected_layer == CoralLayer.L1) {
            // it's at 90deg, driver drives forward while we spin gripper motors negative
            return coral_arm.setGripperVoltage(-3);
        } else if (selected_layer == CoralLayer.L2) {
            // it's at 60deg. needs to rotate down, then driver drives away
            return coral_arm.setAngle(Preset.ScoreL2.getAngle() - 0.0277); // rotate down 10deg
        } else if (selected_layer == CoralLayer.L3) {
            return coral_arm.setAngle(Preset.ScoreL3.getAngle() - 0.0277);
        } else if (selected_layer == CoralLayer.L4) {
            return coral_arm.setAngle(Preset.ScoreL1.getAngle());
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
        return elevator.setHeight(selected_layer.toPreset().getHeight());
    }

    public Command algaeExtractionPrepareArm() {
        return coral_arm.setAngle(Preset.ExtractAlgaeLow.getAngle());
    }
    public Command algaeExtractionExtractAlgae() {
        return coral_arm.setGripperVoltage(-3);
    }
}
