package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ElevatorSubsystem extends SubsystemBase {
    private static ElevatorSubsystem instance;

    protected static final TalonFX leader_motor = new TalonFX(leftMotorID, "canivore"); 
    protected static final TalonFX follower_motor = new TalonFX(rightMotorID, "canivore");
    protected static final MotionMagicVoltage mm_voltage = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
    protected static final Follower follow = new Follower(leftMotorID, false);

    protected static final Supplier<Double> elevator_height = () -> translateRotationsToHeight(leader_motor.getPosition().getValueAsDouble());

    protected ElevatorSubsystem() {
        configureMotors();
    }

    public static synchronized ElevatorSubsystem getInstance() {
        if (instance == null) {
            instance = new ElevatorSubsystem();
        }

        return instance;
    }

    private void configureMotors() {
        BaseStatusSignal.setUpdateFrequencyForAll(250, leader_motor.getPosition(), leader_motor.getVelocity(), leader_motor.getMotorVoltage());

        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Encoder
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        cfg.Feedback.SensorToMechanismRatio = motorGearRatio;
        
        // MotionMagic
        cfg.MotionMagic.MotionMagicAcceleration = motorMaxAcceleration;
        cfg.MotionMagic.MotionMagicCruiseVelocity = motorCruiseVelocity;
        
        // PID + motionmagic constants
        cfg.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        cfg.Slot0.kA = 0.017551;
        cfg.Slot0.kG = 0.32715;
        cfg.Slot0.kS = 0.016181;
        cfg.Slot0.kV = 0.92225;
        cfg.Slot0.kP = 83.066;
        cfg.Slot0.kI = 0;
        cfg.Slot0.kD = 2.3041;
        
        leader_motor.getConfigurator().apply(cfg);
        follower_motor.getConfigurator().apply(cfg);
        
        leader_motor.optimizeBusUtilization();
        
        leader_motor.setPosition(0);
        follower_motor.setPosition(0);
    }

    private static double translateHeightToRotations(double goalHeight) {
        return goalHeight / (22 * 0.25 * 0.0254);
    }
    private static double translateRotationsToHeight(double goalAngle) {
        return (22 * 0.25 * 0.0254) * goalAngle;
    }

    public Trigger isAtHeight(double goalHeight) {
        return new Trigger(() -> MathUtil.isNear(goalHeight, elevator_height.get(), heightTolerance));
    }
    public Trigger isAtHeight(double goalHeight, double tolerance) {
        return new Trigger(() -> MathUtil.isNear(goalHeight, elevator_height.get(), tolerance));
    }

    public Trigger isAboveHeight(double height) {
        return new Trigger(() -> elevator_height.get() > (height + heightTolerance));
    }

    public Command setHeight(double goalHeight) {
        return run(() -> {
            leader_motor.setControl(mm_voltage.withPosition(translateHeightToRotations(goalHeight)));
            follower_motor.setControl(follow);
        });
    }
}
