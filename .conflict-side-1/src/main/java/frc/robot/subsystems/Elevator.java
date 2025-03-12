package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Elevator extends SubsystemBase {
    private static Elevator instance;

    private static final TalonFX leader_motor = new TalonFX(leftMotorID, "canivore"); 
    private static final TalonFX follower_motor = new TalonFX(rightMotorID, "canivore");
    private static final MotionMagicVoltage mm_voltage = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
    private static final Follower follow = new Follower(leftMotorID, false);
    
    // private static CANrange canrange = new CANrange(heightSensorID, "canivore");

    private Elevator() {
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
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Encoder
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        cfg.Feedback.SensorToMechanismRatio = motorGearRatio;

        // MotionMagic
        cfg.MotionMagic.MotionMagicAcceleration = motorMaxAcceleration;
        cfg.MotionMagic.MotionMagicCruiseVelocity = motorCruiseVelocity;

        // PID + motionmagic constants
        cfg.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        cfg.Slot0.kA = 0.01;
        cfg.Slot0.kG = 2.25;
        cfg.Slot0.kS = 0.2;
        cfg.Slot0.kV = 0.0;
        cfg.Slot0.kP = 60;
        cfg.Slot0.kI = 0;
        cfg.Slot0.kD = 0;

        leader_motor.getConfigurator().apply(cfg);
        follower_motor.getConfigurator().apply(cfg);
        
        leader_motor.setPosition(0);
        follower_motor.setPosition(0);
    }

    private static double translateHeightToRotations(double goalHeight) {
        return goalHeight / (2 * Math.PI * sprocketRadius);
    }
    private static double translateRotationsToHeight(double goalAngle) {
        return (2 * Math.PI * sprocketRadius) * goalAngle;
    }
    
    // public double getHeight() {
    //     return canrange.getDistance().getValueAsDouble() / 1000;
    // }

    public double getHeight() {
        return translateRotationsToHeight(leader_motor.getPosition().getValueAsDouble());
    }

    public Trigger isAtHeight(double goalHeight) {
        return new Trigger(() -> MathUtil.isNear(goalHeight, getHeight(), heightTolerance));
    }

    public Command setHeight(double goalHeight) {
        return run(() -> {
            // leader_motor.setPosition(translateHeightToRotations(getHeight()));

            leader_motor.setControl(mm_voltage.withPosition(translateHeightToRotations(goalHeight)));
            follower_motor.setControl(follow);
        });
    }

    public Command stopElevator() {
        return runOnce(() -> {
            leader_motor.setControl(new VoltageOut(0));
        });
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("CANrange Reading (meters)", getHeightNoOffset());
        SmartDashboard.putNumber("Elevator Reported Height (meters)", getHeight());
    }
}
