package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.AlgaeArmConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArm extends SubsystemBase {
    private static AlgaeArm instance;

    private static TalonFX pivotMotor;
    private final PositionVoltage position_voltage = new PositionVoltage(0);

    private static SparkMax gripperMotor;

    private AlgaeArm() {
        pivotMotor = new TalonFX(pivotMotorID, "canivore");
        gripperMotor = new SparkMax(gripperMotorID, MotorType.kBrushless);

        configureMotors();
    }

    public static synchronized AlgaeArm getInstance() {
        if (instance == null) {
            instance = new AlgaeArm();
        }

        return instance;
    }

    private void configureMotors() {
        var cfg = new TalonFXConfiguration();

        // Use internal sensor as feedback source
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        cfg.Feedback.SensorToMechanismRatio = pivotMotorGearRatio;

        // MotionMagic
        cfg.MotionMagic.MotionMagicAcceleration = pivotMotorAcceleration;
        cfg.MotionMagic.MotionMagicCruiseVelocity = pivotMotorCruiseVelocity;

        // PID
        cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        cfg.Slot0.kG = 0.067;
        cfg.Slot0.kP = 30;
        cfg.Slot0.kI = 0;
        cfg.Slot0.kD = 0.3;

        pivotMotor.getConfigurator().apply(cfg);
    }

    // Gripper
    private Command setGripperVoltage(Voltage voltage) {
        return runOnce(() -> gripperMotor.setVoltage(voltage.magnitude()));
    }

    // Pivot
    private Command setPivotAngle(Angle goalAngle) {
        return run(() -> pivotMotor.setControl(
                position_voltage.withPosition(goalAngle.in(Rotations))));
    }

    // sequences
    public Command deployArm() {
        return Commands.parallel(
                setPivotAngle(Degrees.of(35)),
                setGripperVoltage(Volts.of(6)));
    }

    public Command returnArm() {
        return Commands.parallel(
            setPivotAngle(Degrees.of(-10)),
            setGripperVoltage(Volts.of(1)) // to keep the algae in the gripper
        );
    }

    public Command ejectAlgae() {
        return setGripperVoltage(Volts.of(-6))
            .withTimeout(Seconds.of(6))
            .andThen(setGripperVoltage(Volts.of(0)));
    }
}
