package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// CTRE elevator mm class?

public class Elevator extends SubsystemBase {
    private static Elevator instance;

    private static TalonFX leftMotor;
    private static TalonFX rightMotor;
    
    private static CANrange heightSensor;
    private static DigitalInput bottomSensor;

    private Elevator() {
        rightMotor = new TalonFX(rightMotorID, "canivore");
        leftMotor = new TalonFX(leftMotorID, "canivore");
        heightSensor = new CANrange(heightSensorID, "canivore"); // TODO: use CANrange as feedback device into Elevator motors MM
        bottomSensor = new DigitalInput(bottomSensorDIO);

        configureMotors();

        // MotionMagicVoltage
    }

    public static synchronized Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private void configureMotors() {
        var motor_cfg = new TalonFXConfiguration();
        // TODO: max acceleration, max velocity
        motor_cfg.MotionMagic.MotionMagicAcceleration = motorMaxAcceleration;
        motor_cfg.MotionMagic.MotionMagicAcceleration = motorCruiseVelocity;

        // TODO: these are default values! make sure to change
        motor_cfg.Slot0.kA = 0.01;
        motor_cfg.Slot0.kG = 0.067;
        motor_cfg.Slot0.kP = 60;
        motor_cfg.Slot0.kI = 0;
        motor_cfg.Slot0.kD = 0.5;

        // TODO: make them inverted!!!
        rightMotor.getConfigurator().apply(motor_cfg);
        leftMotor.getConfigurator().apply(motor_cfg);
    }
    
    private boolean isAtBottom() {
        return bottomSensor.get();
    }

    public Distance getHeight() {
        return heightSensor.getDistance().getValue();
    }


    // Commands to nudge elevator (doesn't rely on setHeight)
    public Command startNudgeElevator(Voltage volts) {
        return run(() -> {
            rightMotor.setVoltage(volts.magnitude());
            leftMotor.setVoltage(volts.magnitude());
        });
    }

    public Command stopNudgeElevator() {
        return run(() -> {
            rightMotor.setVoltage(0);
            leftMotor.setVoltage(0);
        });
    }

    // TODO: measure elevator height presets
    public enum ElevatorHeightPreset {
        Initial(Inches.of(0)), // not zero!!!
        Intake(Inches.of(0)),
        L1(Inches.of(0)),
        L2(Inches.of(0)),
        L3(Inches.of(0)),
        L4(Inches.of(0));

        private final Distance distance;

        ElevatorHeightPreset(Distance d) {
            distance = d;
        }

        public Distance getHeight() {
            return distance;
        }
    }

    // TODO: write set height function
    public Command setHeight(Distance height) { return Commands.none(); }

    public Command setHeightFromPreset(ElevatorHeightPreset preset) {
        return setHeight(preset.getHeight());
    }
}
