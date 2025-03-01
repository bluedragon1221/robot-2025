package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.AlgaeArmConstants.pivotMotorGearRatio;
import static frc.robot.Constants.CoralArmConstants.*;
import frc.robot.Constants.Preset;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralArm extends SubsystemBase {
    private static CoralArm instance;

    private final PositionVoltage position_voltage = new PositionVoltage(0).withEnableFOC(true);

    private static TalonFX pivotMotor;
    private static CANcoder pivotEncoder;

    private static SparkMax gripperMotor;

    private CoralArm() {
        pivotMotor = new TalonFX(pivotMotorID, "canviore");
        pivotEncoder = new CANcoder(pivotEncoderID, "canivore");

        gripperMotor = new SparkMax(gripperMotorID, MotorType.kBrushless);

        configureMotors();
    }

    public static synchronized CoralArm getInstance() {
        if (instance == null) {
            instance = new CoralArm();
        }

        return instance;
    }

    private void configureMotors() {
        // Pivot encoder
        var encoder_cfg = new MagnetSensorConfigs();
        encoder_cfg.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoder_cfg.MagnetOffset = 0;
        pivotEncoder.getConfigurator().apply(encoder_cfg);

        // Pivot motor
        var pivot_cfg = new TalonFXConfiguration();
        pivot_cfg.Feedback.FeedbackRemoteSensorID = pivotEncoderID;
        pivot_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivot_cfg.Feedback.RotorToSensorRatio = pivotMotorGearRatio;

        pivot_cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivot_cfg.Slot0.kG = 0.06;
        pivot_cfg.Slot0.kP = 2;
        pivot_cfg.Slot0.kI = 0;
        pivot_cfg.Slot0.kD = 0.2;

        pivot_cfg.MotionMagic.MotionMagicAcceleration = pivotMotorAcceleration;
        pivot_cfg.MotionMagic.MotionMagicCruiseVelocity = pivotMotorCruiseVelocity;

        pivot_cfg.CurrentLimits.SupplyCurrentLimit = pivotMotorCurrentLimit.in(Amps);
        pivot_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        pivotMotor.getConfigurator().apply(pivot_cfg);

        // Gripper motor
        var gripper_cfg = new SparkMaxConfig();
        gripper_cfg.idleMode(IdleMode.kBrake);
        gripper_cfg.smartCurrentLimit((int) gripperMotorCurrentLimit.in(Amps));
        gripperMotor.configure(gripper_cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Gripper
    public Command setGripperVoltage(Voltage voltage) {
        return run(() -> gripperMotor.setVoltage(voltage.in(Volts)));
    }

    // Pivot
    private boolean isAtAngle(Angle goalAngle) {
        Angle current_angle = pivotEncoder.getAbsolutePosition().getValue();
        return current_angle.isNear(goalAngle, pivotMotorTolerance);
    }
    private boolean isAtAngle(Angle goalAngle, Angle tolerance) {
        Angle current_angle = pivotEncoder.getAbsolutePosition().getValue();
        return current_angle.isNear(goalAngle, tolerance);
    }

    private Command setPivotAngle(Angle goalAngle) {
        return run(() -> pivotMotor.setControl(
            position_voltage.withPosition(goalAngle.in(Rotations))));
    }

    public Command setPivotAngleFromPreset(Preset preset) {
        return setPivotAngle(preset.getAngle());
    }

    public boolean isAtAngleFromPreset(Preset preset) {
        return isAtAngle(preset.getAngle());
    }
    public boolean isAtAngleFromPreset(Preset preset, Angle tolerance) {
        return isAtAngle(preset.getAngle(), tolerance);
    }
}
