package frc.robot.subsystems.coralarmpivot;

import static frc.robot.subsystems.coralarmpivot.CoralArmPivotConstants.pivotEncoderID;
import static frc.robot.subsystems.coralarmpivot.CoralArmPivotConstants.pivotEncoderOffset;
import static frc.robot.subsystems.coralarmpivot.CoralArmPivotConstants.pivotMotorAcceleration;
import static frc.robot.subsystems.coralarmpivot.CoralArmPivotConstants.pivotMotorCruiseVelocity;
import static frc.robot.subsystems.coralarmpivot.CoralArmPivotConstants.pivotMotorCurrentLimit;
import static frc.robot.subsystems.coralarmpivot.CoralArmPivotConstants.pivotMotorGearRatio;
import static frc.robot.subsystems.coralarmpivot.CoralArmPivotConstants.pivotMotorID;
import static frc.robot.subsystems.coralarmpivot.CoralArmPivotConstants.pivotMotorTolerance;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CoralArmPivotSubsystem extends SubsystemBase {
    private static CoralArmPivotSubsystem instance;

    protected static final TalonFX pivot_motor = new TalonFX(pivotMotorID, "canivore");
    protected static final CANcoder pivot_encoder = new CANcoder(pivotEncoderID, "canivore");
    private static final MotionMagicVoltage pivot_mm_voltage = new MotionMagicVoltage(0).withEnableFOC(true);

    protected static final Supplier<Double> pivot_angle = () -> pivot_encoder.getAbsolutePosition().getValueAsDouble();

    protected CoralArmPivotSubsystem() {
        configureMotors();
    }

    public static synchronized CoralArmPivotSubsystem getInstance() {
        if (instance == null) {
            instance = new CoralArmPivotSubsystem();
        }

        return instance;
    }

    private void configureMotors() {
        // Encoder
        BaseStatusSignal.setUpdateFrequencyForAll(50, pivot_encoder.getPosition(), pivot_encoder.getVelocity());
        pivot_encoder.optimizeBusUtilization();

        var encoder_cfg = new MagnetSensorConfigs();
        encoder_cfg.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoder_cfg.MagnetOffset = pivotEncoderOffset;
        pivot_encoder.getConfigurator().apply(encoder_cfg);

        // Motor
        BaseStatusSignal.setUpdateFrequencyForAll(50, pivot_motor.getPosition(), pivot_motor.getVelocity(), pivot_motor.getMotorVoltage(), pivot_motor.getRotorVelocity(), pivot_motor.getRotorPosition());
        pivot_motor.optimizeBusUtilization();

        var pivot_cfg = new TalonFXConfiguration();
        pivot_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivot_cfg.Feedback.FeedbackRemoteSensorID = pivotEncoderID;
        pivot_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivot_cfg.Feedback.RotorToSensorRatio = pivotMotorGearRatio;

        pivot_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivot_cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivot_cfg.Slot0.kA = 0.26315;
        pivot_cfg.Slot0.kG = 0.33;
        pivot_cfg.Slot0.kS = 0.10091;
        pivot_cfg.Slot0.kV = 4.4567;
        pivot_cfg.Slot0.kP = 68.82;
        pivot_cfg.Slot0.kI = 0;
        pivot_cfg.Slot0.kD = 8.8037;

        pivot_cfg.MotionMagic.MotionMagicAcceleration = pivotMotorAcceleration;
        pivot_cfg.MotionMagic.MotionMagicCruiseVelocity = pivotMotorCruiseVelocity;

        pivot_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivot_cfg.CurrentLimits.SupplyCurrentLimit = pivotMotorCurrentLimit;

        pivot_motor.getConfigurator().apply(pivot_cfg);
    }


    public Trigger isAtAngle(double goalAngle) {
        return new Trigger(() -> MathUtil.isNear(goalAngle, pivot_angle.get(), pivotMotorTolerance));
    }

    public Trigger isBeyondAngle(double angle) {
        return new Trigger(() -> pivot_angle.get() > angle);
    }

    public Trigger isBetweenAngles(double min, double max) {
        return new Trigger(() -> min < pivot_angle.get() && pivot_angle.get() < max);
    }

    public Command setAngle(double goalAngle) {
        return run(() -> {
            pivot_motor.setControl(
                pivot_mm_voltage.withPosition(goalAngle)
            );
        });
    }
}
