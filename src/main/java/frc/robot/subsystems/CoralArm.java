package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.AlgaeArmConstants.pivotMotorGearRatio;
import static frc.robot.Constants.CoralArmConstants.gripperMotorCurrentLimit;
import static frc.robot.Constants.CoralArmConstants.gripperMotorID;
import static frc.robot.Constants.CoralArmConstants.pivotEncoderID;
import static frc.robot.Constants.CoralArmConstants.pivotMotorAcceleration;
import static frc.robot.Constants.CoralArmConstants.pivotMotorCruiseVelocity;
import static frc.robot.Constants.CoralArmConstants.pivotMotorCurrentLimit;
import static frc.robot.Constants.CoralArmConstants.pivotMotorID;
import static frc.robot.Constants.CoralArmConstants.pivotMotorTolerance;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import frc.robot.Constants.Preset;

public class CoralArm extends SubsystemBase {
    private static CoralArm instance;

    private static final TalonFX pivot_motor = new TalonFX(pivotMotorID, "canivore");
    private static final CANcoder pivot_encoder = new CANcoder(pivotEncoderID, "canivore");
    private static final MotionMagicVoltage pivot_position_voltage = new MotionMagicVoltage(0).withEnableFOC(true);
    private static Angle pivot_goal_angle;

    private final SparkMax gripper_motor = new SparkMax(gripperMotorID, MotorType.kBrushless);

    private CoralArm() {
        pivot_goal_angle = Preset.Initial.getAngle();

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
        pivot_encoder.getConfigurator().apply(encoder_cfg);

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

        pivot_motor.getConfigurator().apply(pivot_cfg);

        // Gripper motor
        var gripper_cfg = new SparkMaxConfig();
        gripper_cfg.idleMode(IdleMode.kBrake);
        gripper_cfg.smartCurrentLimit((int) gripperMotorCurrentLimit.in(Amps));
        gripper_motor.configure(gripper_cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Gripper
    public Command setGripperVoltage(Voltage voltage) {
        return run(() -> gripper_motor.setVoltage(voltage.in(Volts)));
    }

    // Pivot
    private boolean isAtAngle(Angle goalAngle) {
        Angle current_angle = pivot_encoder.getAbsolutePosition().getValue();
        return current_angle.isNear(goalAngle, pivotMotorTolerance);
    }
    private boolean isAtAngle(Angle goalAngle, Angle tolerance) {
        Angle current_angle = pivot_encoder.getAbsolutePosition().getValue();
        return current_angle.isNear(goalAngle, tolerance);
    }
    public boolean isAtAngleFromPreset(Preset preset) {
        return isAtAngle(preset.getAngle());
    }
    public boolean isAtAngleFromPreset(Preset preset, Angle tolerance) {
        return isAtAngle(preset.getAngle(), tolerance);
    }

    private void setPivotAngle(Angle goalAngle) {
        pivot_goal_angle = goalAngle;
    }
    public void setPivotAngleFromPreset(Preset preset) {
        setPivotAngle(preset.getAngle());
    }

    @Override
    public void periodic() {
        pivot_motor.setControl(
            pivot_position_voltage.withPosition(pivot_goal_angle.in(Rotations))
        );
    }
}
