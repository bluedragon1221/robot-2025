package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        bottomSensor = new DigitalInput(bottomSensorGPIO);

        configureMotors();
    }

    public static synchronized Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private void configureMotors() {
        var motor_cfg = new TalonFXConfiguration();
        motor_cfg.MotionMagic.MotionMagicAcceleration = motorMaxAcceleration;
        motor_cfg.MotionMagic.MotionMagicAcceleration = motorCruiseVelocity;

        // TODO: these are default values! make sure to change
        var slot0 = motor_cfg.Slot0;
        slot0.kS = 0.25;
        slot0.kV = 0.12;
        slot0.kA = 0.01;
        slot0.kP = 60;
        slot0.kI = 0;
        slot0.kD = 0.5;

        rightMotor.getConfigurator().apply(motor_cfg);
        leftMotor.getConfigurator().apply(motor_cfg);
    }
    
    public boolean isAtBottom() {
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
    public enum HeightPreset {
        Down(Inches.of(0)),
        Intake(Inches.of(0)),
        L1(Inches.of(0)),
        L2(Inches.of(0)),
        L3(Inches.of(0)),
        L4(Inches.of(0));

        private final Distance distance;

        HeightPreset(Distance d) {
            distance = d;
        }

        public Distance getHeight() {
            return distance;
        }
    }

    // TODO: write set height function
    public Command setHeight(Distance height) { return Commands.none(); }

    public Command setHeightFromPreset(HeightPreset preset) {
        return setHeight(preset.getHeight());
    }
}
