package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.ElevatorConstants.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Preset;

public class Elevator extends SubsystemBase {
    private static Elevator instance;

    private static final TalonFX leader_motor = new TalonFX(leftMotorID, "canivore"); 
    private static final TalonFX follower_motor = new TalonFX(rightMotorID, "canivore");
    private static final MotionMagicVoltage position_voltage = new MotionMagicVoltage(0).withEnableFOC(true);
    private static Distance goal_height;

    private static CANrange canrange = new CANrange(heightSensorID, "canivore");

    private static class TuneableConstants {
        private static double kA = 0.01;
        private static double kG = 0.067;
        private static double kS = 0;
        private static double kP = 60;
        private static double kI = 0;
        private static double kD = 0.5;

        public static void initDashboard() {
            SmartDashboard.putNumber("Elevator/kA", kA);
            SmartDashboard.putNumber("Elevator/kG", kG);
            SmartDashboard.putNumber("Elevator/kS", kS);
            SmartDashboard.putNumber("Elevator/kP", kP);
            SmartDashboard.putNumber("Elevator/kI", kI);
            SmartDashboard.putNumber("Elevator/kD", kD);
        }

        public static boolean updateDashboard() {
            double newKA = SmartDashboard.getNumber("CoralArm/kA", kA);
            double newKG = SmartDashboard.getNumber("CoralArm/kG", kG);
            double newKS = SmartDashboard.getNumber("CoralArm/kS", kS);
            double newKP = SmartDashboard.getNumber("CoralArm/kP", kP);
            double newKI = SmartDashboard.getNumber("CoralArm/kI", kI);
            double newKD = SmartDashboard.getNumber("CoralArm/kD", kD);
            
            // Check if any values have changed
            boolean changed = newKA != kA || newKG != kG|| newKS != kS ||
                newKP != kP || newKI != kI || newKD != kD;
            
            // Update stored values
            kA = newKA;
            kG = newKG;
            kS = newKS;
            kP = newKP;
            kI = newKI;
            kD = newKD;
            
            return changed;
        }
    }

    private Elevator() {
        // initial goal height (when the robot starts)
        goal_height = Preset.Initial.getHeight();

        TuneableConstants.initDashboard();

        configureMotors();
    }

    public static synchronized Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private void configureMotors() {
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Encoder
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        cfg.Feedback.SensorToMechanismRatio = motorGearRatio;

        // MotionMagic
        cfg.MotionMagic.MotionMagicAcceleration = motorMaxAcceleration;
        cfg.MotionMagic.MotionMagicCruiseVelocity = motorCruiseVelocity;

        // PID + motionmagic constants
        cfg.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        cfg.Slot0.kA = TuneableConstants.kA;
        cfg.Slot0.kG = TuneableConstants.kG;
        cfg.Slot0.kS = TuneableConstants.kS;
        cfg.Slot0.kP = TuneableConstants.kP;
        cfg.Slot0.kI = TuneableConstants.kI;
        cfg.Slot0.kD = TuneableConstants.kD;

        leader_motor.getConfigurator().apply(cfg);
        follower_motor.getConfigurator().apply(cfg);

        follower_motor.setControl(new Follower(leftMotorID, false));
    }

    // // Commands to nudge elevator (doesn't rely on setHeight)
    // public Command startNudgeElevator(Voltage volts) {
    //     return run(() -> {
    //         leader_motor.setVoltage(volts.in(Volts));
    //     });
    // }

    // public Command stopNudgeElevator() {
    //     return runOnce(() -> {
    //         leader_motor.setVoltage(0);
    //     });
    // }

    public Distance getHeight() {
        return canrange.getDistance().getValue().minus(canrangeOffset); // subtracts canrangeOffset to get the "actual" position
    }

    private BooleanSupplier isAtHeight(Distance goalHeight) {
        return () -> getHeight().isNear(goalHeight, heightTolerance);
    }
    private boolean isAtHeight(Distance goalHeight, Distance tolerance) {
        return getHeight().isNear(goalHeight, tolerance);
    }
    public BooleanSupplier isAtHeightFromPreset(Preset preset) {
        return isAtHeight(preset.getHeight());
    }
    public boolean isAtHeightFromPreset(Preset preset, Distance tolerance) {
        return isAtHeight(preset.getHeight(), tolerance);
    }

    private static Angle translateHeightToRotations(Distance goalHeight) {
        return Radians.of(goalHeight.in(Inches) / (2 * Math.PI * sprocketRadius.in(Inches)));
    }

    private void setHeight(Distance goalHeight) {
        goal_height = goalHeight;
    }

    public void setHeightFromPreset(Preset preset) {
        setHeight(preset.getHeight());
    }

    @Override
    public void periodic() {
        var curr_estimate_rotations = translateHeightToRotations(getHeight());
        var curr_actual_rotations = leader_motor.getPosition().getValue();
        if (!curr_actual_rotations.isNear(curr_estimate_rotations, angleTolerance)) {
            leader_motor.setPosition(curr_estimate_rotations);
        }

        Angle goalRotations = translateHeightToRotations(goal_height);
        leader_motor.setControl(position_voltage.withPosition(goalRotations));

        SmartDashboard.putString("CANrange Reading", String.format("%.9f", canrange.getDistance().getValue().in(Millimeters)));
    
        boolean changed = TuneableConstants.updateDashboard();
        if (changed) {
            configureMotors();
        }
    }
}
