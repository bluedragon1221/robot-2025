package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CoralArmPivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class CoralArmPivot extends SubsystemBase {
    private static CoralArmPivot instance;

    private static final TalonFX pivot_motor = new TalonFX(pivotMotorID, "canivore");
    private static final CANcoder pivot_encoder = new CANcoder(pivotEncoderID, "canivore");
    private static final MotionMagicVoltage pivot_mm_voltage = new MotionMagicVoltage(0).withEnableFOC(true);

    private final Trigger atMax = new Trigger(() -> MathUtil.isNear(pivot_motor.getPosition().getValueAsDouble(), 0.23, 0.01));
    private final Trigger atMin = new Trigger(() -> MathUtil.isNear(pivot_motor.getPosition().getValueAsDouble(), -0.23, 0.01));

    private static class TuneableConstants {
        private static double kG = 0.06;
        private static double kP = 0.12;
        private static double kI = 0;
        private static double kD = 0.01;

        public static void initDashboard() {
            SmartDashboard.putNumber("CoralArm/kG", kG);
            SmartDashboard.putNumber("CoralArm/kP", kP);
            SmartDashboard.putNumber("CoralArm/kI", kI);
            SmartDashboard.putNumber("CoralArm/kD", kD);
        }
        
        // Method to update constants from SmartDashboard
        public static boolean updateDashboard() {
            double newKG = SmartDashboard.getNumber("CoralArm/kG", kG);
            double newKP = SmartDashboard.getNumber("CoralArm/kP", kP);
            double newKI = SmartDashboard.getNumber("CoralArm/kI", kI);
            double newKD = SmartDashboard.getNumber("CoralArm/kD", kD);
            
            // Check if any values have changed
            boolean changed = newKG != kG|| newKP != kP || newKI != kI || newKD != kD;
            
            // Update stored values
            kG = newKG;
            kP = newKP;
            kI = newKI;
            kD = newKD;
            
            return changed;
        }
    }

    private CoralArmPivot() {
        configureMotors();

        BaseStatusSignal.setUpdateFrequencyForAll(250, pivot_motor.getPosition(), pivot_motor.getVelocity(), pivot_motor.getMotorVoltage(), pivot_motor.getRotorVelocity(), pivot_motor.getRotorPosition());
        BaseStatusSignal.setUpdateFrequencyForAll(250, pivot_encoder.getPosition(), pivot_encoder.getVelocity());

        pivot_motor.optimizeBusUtilization();
        pivot_encoder.optimizeBusUtilization();

        SmartDashboard.putNumber("Set Coral Arm Pivot", 0);

        SignalLogger.start();

        TuneableConstants.initDashboard();
    }

    private final SysIdRoutine      m_sysIdRoutine   =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(Volts.per(Second).of(0.5),
                                    Volts.of(2),
                                    null,
                                    state -> SignalLogger.writeString("stateA", state.toString())),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motor(s).
                output -> {
                    pivot_motor.setControl(new VoltageOut(output));
                },
                // Tell SysId how to record a frame of data for each motor on the mechanism being
                // characterized.
                null,
                this));

    public static synchronized CoralArmPivot getInstance() {
        if (instance == null) {
            instance = new CoralArmPivot();
        }

        return instance;
    }    

    private void configureMotors() {
        // Encoder
        var encoder_cfg = new MagnetSensorConfigs();
        encoder_cfg.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoder_cfg.MagnetOffset = pivotEncoderOffset;
        pivot_encoder.getConfigurator().apply(encoder_cfg);

        // Motor
        var pivot_cfg = new TalonFXConfiguration();
        pivot_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivot_cfg.Feedback.FeedbackRemoteSensorID = pivotEncoderID;
        pivot_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivot_cfg.Feedback.RotorToSensorRatio = pivotMotorGearRatio;

        pivot_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivot_cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivot_cfg.Slot0.kA = 0.26315;
        pivot_cfg.Slot0.kG = 0.39575;
        pivot_cfg.Slot0.kS = 0.10091;
        pivot_cfg.Slot0.kV = 4.4567;
        pivot_cfg.Slot0.kP = 68.814;
        pivot_cfg.Slot0.kI = 0;
        pivot_cfg.Slot0.kD = 8.8037;

        pivot_cfg.MotionMagic.MotionMagicAcceleration = pivotMotorAcceleration;
        pivot_cfg.MotionMagic.MotionMagicCruiseVelocity = pivotMotorCruiseVelocity;

        pivot_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivot_cfg.CurrentLimits.SupplyCurrentLimit = pivotMotorCurrentLimit;

        pivot_motor.getConfigurator().apply(pivot_cfg);
    }

    public void reconfigurePivotMotor() {
        TuneableConstants.updateDashboard();
        System.out.println("Reconfigured coral pivot");

        var pivot_cfg = new TalonFXConfiguration();
        pivot_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivot_cfg.Feedback.FeedbackRemoteSensorID = pivotEncoderID;
        pivot_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivot_cfg.Feedback.RotorToSensorRatio = pivotMotorGearRatio;

        pivot_cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivot_cfg.Slot0.kG = TuneableConstants.kG;
        pivot_cfg.Slot0.kP = TuneableConstants.kP;
        pivot_cfg.Slot0.kI = TuneableConstants.kI;
        pivot_cfg.Slot0.kD = TuneableConstants.kD;

        pivot_cfg.MotionMagic.MotionMagicAcceleration = pivotMotorAcceleration;
        pivot_cfg.MotionMagic.MotionMagicCruiseVelocity = pivotMotorCruiseVelocity;

        pivot_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivot_cfg.CurrentLimits.SupplyCurrentLimit = pivotMotorCurrentLimit;

        pivot_motor.getConfigurator().apply(pivot_cfg);

        System.out.println("Reconfigured CoralArmPivot");
    }

    private double getAngle() {
        return pivot_encoder.getAbsolutePosition().getValueAsDouble();
    }

    public Trigger isAtAngle(double goalAngle) {
        return new Trigger(() -> MathUtil.isNear(goalAngle, getAngle(), pivotMotorTolerance));
    }

    public Command setAngle(double goalAngle) {
        return run(() -> {
            pivot_motor.setControl(
                pivot_mm_voltage.withPosition(goalAngle)
            );
        });
    }

    public Command setAngleFromDashboard() {
        return run(() -> {
            pivot_motor.setControl(
                pivot_mm_voltage.withPosition(SmartDashboard.getNumber("Set Coral Arm Pivot", 0))
            );
        });
    }

    public Command runSysICommand() {
        return (m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(atMax))
                .andThen(m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(atMin))
                .andThen(m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(atMax))
                .andThen(m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(atMin))
                .andThen(Commands.print("DONE"));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Motor", pivot_motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Motor Velocity", pivot_motor.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Pivot Encoder", pivot_encoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Encoder Velocity", pivot_encoder.getVelocity().getValueAsDouble());
    }
}