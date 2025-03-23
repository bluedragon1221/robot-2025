package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Elevator extends SubsystemBase {
    private static Elevator instance;

    private static final TalonFX leader_motor = new TalonFX(leftMotorID, "canivore"); 
    private static final TalonFX follower_motor = new TalonFX(rightMotorID, "canivore");
    private static final MotionMagicVoltage mm_voltage = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
    private static final Follower follow = new Follower(leftMotorID, false);
    
    @SuppressWarnings("unused")
    private final Trigger atMax = new Trigger(() -> MathUtil.isNear(getHeight(), 0.58, 0.0254));

    @SuppressWarnings("unused")
    private final Trigger atMin = new Trigger(() -> MathUtil.isNear(getHeight(), 0, 0.0254));

    // SysID Routine
    // private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    //     // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
    //     new SysIdRoutine.Config(
    //         Volts.per(Second).of(1),
    //         Volts.of(4),
    //         null,
    //         state -> SignalLogger.writeString("state", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         // Tell SysId how to plumb the driving voltage to the motor(s).
    //         output -> {
    //             leader_motor.setControl(new VoltageOut(output));
    //             follower_motor.setControl(follow);
    //         },
    //         // Tell SysId how to record a frame of data for each motor on the mechanism being
    //         // characterized.
    //         null,
    //         this
    //     )
    // );

    private Elevator() {
        configureMotors();
    }

    public static synchronized Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }
    
    public static class ElevatorHeight {
        public static final double initial = 0;
        public static final double storage = 0;
        public static final double scoreL1 = 0.1524;
        public static final double scoreL2 = 0.15;
        public static final double scoreL3 = 0.3556;
        public static final double scoreL4 = 0.6604;
        public static final double extractAlgaeLow = 0.34;
        public static final double extractAlgaeHigh = 0.5;
        public static final double intakeCatch = 0.5148;
        public static final double postIntakeCatch = 0.517;
        public static final double intakeGrip = 0.4387;
        public static final double scoreProcessor = 0.1224;
        public static final double scoreBarge = 0.6850;
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

    public double getHeight() {
        return translateRotationsToHeight(leader_motor.getPosition().getValueAsDouble());
    }

    public Trigger isAtHeight(double goalHeight) {
        return new Trigger(() -> MathUtil.isNear(goalHeight, getHeight(), heightTolerance));
    }
    public Trigger isAtHeight(double goalHeight, double tolerance) {
        return new Trigger(() -> MathUtil.isNear(goalHeight, getHeight(), tolerance));
    }

    public Trigger isGreaterThanHeight(double height) {
        return new Trigger(() -> getHeight() > (height + heightTolerance));
    }

    public Command setHeight(double goalHeight) {
        return run(() -> {
            leader_motor.setControl(mm_voltage.withPosition(translateHeightToRotations(goalHeight)));
            follower_motor.setControl(follow);
        });
    }

    public Command stopElevator() {
        return runOnce(() -> {
            leader_motor.setControl(new VoltageOut(0));
            follower_motor.setControl(follow);
        });
    }

    // public Command runSysICommand() {
    //     return (sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(atMax))
    //             .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(atMin))
    //             .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(atMax))
    //             .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(atMin))
    //             .andThen(Commands.print("DONE"));
    // }
}
