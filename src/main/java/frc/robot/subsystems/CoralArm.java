package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.CoralArmConstants.*;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralArm extends SubsystemBase {
    private static TalonFX pivotMotor;
    private static SparkMax gripperMotor;
    private static CANcoder pivotEncoder;

    public CoralArm() {
        pivotMotor = new TalonFX(pivotMotorID, "canbus");
        pivotEncoder = new CANcoder(pivotEncoderID, "canivore");
        gripperMotor = new SparkMax(gripperMotorID, MotorType.kBrushless);

        configureMotors();
    }

    private void configureMotors() {
        // -- Configure pivot encoder --//
        var encoder_cfg = new MagnetSensorConfigs();
        encoder_cfg.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoder_cfg.MagnetOffset = 0; // TODO pivot encoder magnet offset
        pivotEncoder.getConfigurator().apply(encoder_cfg);

        // -- Configure pivot motor --//
        var pivot_cfg = new TalonFXConfiguration();
        pivot_cfg.Feedback.FeedbackRemoteSensorID = pivotEncoderID;
        pivot_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivot_cfg.Feedback.RotorToSensorRatio = 1.0;

        pivot_cfg.MotionMagic.MotionMagicAcceleration = pivotMotorAcceleration;
        pivot_cfg.MotionMagic.MotionMagicCruiseVelocity = pivotMotorCruiseVelocity;

        pivot_cfg.CurrentLimits.SupplyCurrentLimit = 40;
        pivot_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        pivotMotor.getConfigurator().apply(pivot_cfg);

        // -- Configure gripper motor --//
        var gripper_cfg = new SparkMaxConfig();
        gripper_cfg.idleMode(IdleMode.kBrake);
        // TODO more config for gripper
        gripperMotor.configure(gripper_cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private Rotation2d getPivotAngle() {
        return Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition().getValue().in(Rotations));
    }

    private boolean isAtAngle(Rotation2d target_angle) {
        var diff = getPivotAngle().minus(target_angle).getDegrees();
        return Math.abs(diff) < pivotMotorTolerance;
    }

    // TODO figure out how the heck this thing is supposed to work
    public Command setPivotAngle(Rotation2d target_angle) {
        return Commands.none();
    }

    public enum PivotPreset {
        Down(Rotation2d.fromDegrees(0)),
        L4(Rotation2d.fromDegrees(90)),
        L3(Rotation2d.fromDegrees(125));

        private final Rotation2d angle;

        PivotPreset(Rotation2d a) {
            angle = a;
        }

        public Rotation2d getAngle() {
            return angle;
        }
    }

    public Command setPivotAngleFromPreset(PivotPreset preset) {
        return setPivotAngle(preset.getAngle());
    }

    public Command setGripperVoltage(double voltage) {
        return run(() -> gripperMotor.setVoltage(voltage));
    }

    public Command seqStopGripper() {
        return setGripperVoltage(0);
    }

    public Command seqIntake() {
        Time timeout = Milliseconds.of(100);

        return setGripperVoltage(70)
            .andThen(Commands.waitTime(timeout))
            .andThen(this::seqStopGripper);
    }
}
