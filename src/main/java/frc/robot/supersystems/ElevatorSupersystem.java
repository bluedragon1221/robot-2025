package frc.robot.supersystems;

import static frc.robot.Constants.ElevatorSupersystemConstants.beamBreakSensorDIO;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Preset;
import frc.robot.subsystems.CoralArmGripper;
import frc.robot.subsystems.CoralArmPivot;
import frc.robot.subsystems.Elevator;

public class ElevatorSupersystem {
    private static ElevatorSupersystem instance;

    private static final Elevator elevator = Elevator.getInstance();
    private static final CoralArmGripper coral_arm_gripper = CoralArmGripper.getInstance();
    private static final CoralArmPivot coral_arm_pivot = CoralArmPivot.getInstance();

    private double cur_elevator_height = 0.0;
    private double cur_arm_angle = 0.0;
    private double cur_gripper_voltage = 0.0;

    private static final DigitalInput beam_break_sensor = new DigitalInput(beamBreakSensorDIO);
    private static final Trigger hasCoral = new Trigger(() -> beam_break_sensor.get());

    public Command setState(double elevator_height, double arm_angle, double gripper_voltage) {
        cur_elevator_height = elevator_height;
        cur_arm_angle = arm_angle;
        cur_gripper_voltage = gripper_voltage;
        return Commands.parallel(
                elevator.setHeight(elevator_height),
                coral_arm_pivot.setAngle(arm_angle),
                coral_arm_gripper.setGripperVoltage(gripper_voltage));
    }

    public Command setState(double elevator_height, double arm_angle) {
        cur_elevator_height = elevator_height;
        cur_arm_angle = arm_angle;
        return setState(elevator_height, arm_angle, cur_gripper_voltage);
    }

    public Command setStateElevator(double elevator_height) {
        cur_elevator_height = elevator_height;
        return setState(elevator_height, cur_arm_angle, cur_gripper_voltage);
    }

    public Command setStateGripper(double gripper_voltage) {
        cur_gripper_voltage = gripper_voltage;
        return setState(cur_elevator_height, cur_arm_angle, gripper_voltage);
    }

    public Command setStatePivot(double arm_angle) {
        cur_arm_angle = arm_angle;
        return setState(cur_elevator_height, arm_angle, cur_gripper_voltage);
    }

    public Command setStatePreset(Preset preset) {
        cur_elevator_height = preset.getHeight();
        cur_arm_angle = preset.getAngle();
        return setState(preset.getHeight(), preset.getAngle(), cur_gripper_voltage);
    }

    public Command setStatePreset(Preset preset, double gripper_voltage) {
        cur_elevator_height = preset.getHeight();
        cur_arm_angle = preset.getAngle();
        cur_gripper_voltage = gripper_voltage;
        return setState(preset.getHeight(), preset.getAngle(), gripper_voltage);
    }

    public Command setStateFromDashboard() {
        return Commands.parallel(
                elevator.setHeightFromDashboard(),
                coral_arm_pivot.setAngleFromDashboard(),
                coral_arm_gripper.setVoltageFromDashboard());
    }

    public static synchronized ElevatorSupersystem getInstance() {
        if (instance == null) {
            instance = new ElevatorSupersystem();
        }

        return instance;
    }

    public Command  storagePosition() {
        return setStatePivot(Preset.Storage.getAngle())
            .until(coral_arm_pivot.isGreaterThanAngle(0))
            .andThen(setStatePreset(Preset.Storage));
    }

    // INTAKE
    public Command intakePrepare() {
        return setStatePivot(0)
            .until(coral_arm_pivot.isAtAngle(0))
            .andThen(setStateElevator(Preset.IntakeCatch.getHeight()))
            .until(elevator.isAtHeight(Preset.IntakeCatch.getHeight()))
            .andThen(setStatePreset(Preset.IntakeCatch))
        .onlyIf(hasCoral.negate()); // only run of we don't already have a coral
    }

    public Command intakeLoad() {
        return setStatePreset(Preset.IntakeGrip, 2)
            .until(hasCoral)
            .withTimeout(3)
            .andThen(setStatePreset(Preset.IntakeCatch, 1))
            .onlyIf(elevator.isAtHeight(Preset.IntakeCatch.getHeight())
            .and(coral_arm_pivot.isAtAngle(Preset.IntakeCatch.getAngle()))
            .and(hasCoral.negate())); // don't try to intake with a low elevator
    }

    public Command intakePost() {
        return setStateElevator(Preset.IntakeCatch.getHeight()+0.02) // we only pivot a bit higher than IntakeCatch to be safe
            .until(elevator.isAtHeight(Preset.IntakeCatch.getHeight()+0.02))
            .andThen(setStatePivot(Preset.Storage.getAngle()))
            .until(coral_arm_pivot.isGreaterThanAngle(0.1))
            .andThen(setStatePreset(Preset.Storage));
    }

    // SCORE CORAL
    public Command coralPrepareL4() {
        return setStatePivot(Preset.ScoreL4.getAngle())
            .until(coral_arm_pivot.isGreaterThanAngle(0))
            .andThen(setStatePreset(Preset.ScoreL4))
        .onlyIf(hasCoral);
    }

    public Command coralPrepareL3() {
        return setStatePivot(Preset.ScoreL3.getAngle())
            .until(coral_arm_pivot.isGreaterThanAngle(0))
            .andThen(setStatePreset(Preset.ScoreL3))
        .onlyIf(hasCoral);
    }

    public Command coralPrepareL2() {
        return setStatePivot(Preset.ScoreL2.getAngle())
            .until(coral_arm_pivot.isGreaterThanAngle(0))
            .andThen(setStatePreset(Preset.ScoreL3))
        .onlyIf(hasCoral);
    }

    public Command coralPrepareL1() {
        return setStatePivot(0)
            .until(coral_arm_pivot.isAtAngle(0))
            .andThen(setStateElevator(Preset.ScoreL1.getHeight()))
        .onlyIf(hasCoral);
    }

    public Command coralScoreL4() {
        return setState(Preset.ScoreL4.getHeight(), 0)
            .onlyIf(elevator.isAtHeight(Preset.ScoreL4.getHeight(), 0.02)
                .and(coral_arm_pivot.isAtAngle(Preset.ScoreL4.getAngle()))
                .and(hasCoral));
    }

    public Command coralScoreL3() {
        return setState(Preset.ScoreL3.getHeight(), Preset.ScoreL3.getAngle() - 0.0277)
            .onlyIf(elevator.isAtHeight(Preset.ScoreL3.getHeight(), 0.02)
                .and(coral_arm_pivot.isAtAngle(Preset.ScoreL3.getAngle()))
                .and(hasCoral));
    }

    public Command coralScoreL2() {
        return setState(Preset.ScoreL2.getHeight(), Preset.ScoreL2.getAngle() - 0.0277)
            .onlyIf(elevator.isAtHeight(Preset.ScoreL2.getHeight(), 0.02)
                .and(coral_arm_pivot.isAtAngle(Preset.ScoreL2.getAngle()))
                .and(hasCoral));
    }

    public Command coralScoreL1() {
        return setStateGripper(-0.5)
            .onlyIf(elevator.isAtHeight(Preset.ScoreL1.getHeight(), 0.02)
                .and(coral_arm_pivot.isAtAngle(0))
                .and(hasCoral));
    }

    // EXTRACT ALGAE
    public Command extractionPrepareLow() {
        return setStatePivot(0)
            .until(coral_arm_pivot.isAtAngle(0))
            .andThen(setStatePreset(Preset.ExtractAlgaeLow))
        .onlyIf(hasCoral.negate()); // can't run if we already have a coral
    }

    public Command extractionPrepareHigh() {
        return setStatePivot(0)
            .until(coral_arm_pivot.isAtAngle(0))
            .andThen(setStatePreset(Preset.ExtractAlgaeHigh))
        .onlyIf(hasCoral.negate());
    }

    public Command extractionExtractLow() {
        return setStateGripper(12)
            .until(hasCoral) // TODO: does the coral trigger the beam break?
            .withTimeout(1)
        .onlyIf(elevator.isAtHeight(Preset.ExtractAlgaeLow.getHeight())
            .and(coral_arm_pivot.isAtAngle(Preset.ExtractAlgaeLow.getAngle())));
    }

    public Command extractionExtractHigh() {
        return setStateGripper(12)
            .until(hasCoral)
            .withTimeout(1)
        .onlyIf(elevator.isAtHeight(Preset.ExtractAlgaeLow.getHeight())
            .and(coral_arm_pivot.isAtAngle(Preset.ExtractAlgaeLow.getAngle())));
    }
}
