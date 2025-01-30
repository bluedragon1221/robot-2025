package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private static TalonFX leftMotor;
    private static TalonFX rightMotor;
    
    private static CANrange heightSensor;
    private static DigitalInput bottomSensor;

    public Elevator() {
        rightMotor = new TalonFX(rightMotorID, "canivore");
        leftMotor = new TalonFX(leftMotorID, "canivore");
        heightSensor = new CANrange(heightSensorID, "canivore");
        bottomSensor = new DigitalInput(bottomSensorGPIO);

        configureMotors();
    }

    private void configureMotors() {
        var motor_cfg = new TalonFXConfiguration();
        motor_cfg.MotionMagic.MotionMagicAcceleration = motorAcceleration;
        motor_cfg.MotionMagic.MotionMagicAcceleration = motorCruiseVelocity;

        var right_motor_cfg = motor_cfg;
        right_motor_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor.getConfigurator().apply(right_motor_cfg);
        
        var left_motor_cfg = motor_cfg;
        left_motor_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotor.getConfigurator().apply(right_motor_cfg);
    }
    
    public boolean isAtBottom() {
        return bottomSensor.get();
    }

    public Distance getHeight() {
        return heightSensor.getDistance().getValue();
    }

    // TODO write set height function
    public Command setHeight(Distance height) { return Commands.none(); }
}
