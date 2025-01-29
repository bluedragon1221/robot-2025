package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import static frc.robot.Constants.ElevatorConstants.*;

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
        // bottomSensor = new DigitalInput(); // TODO initialize bottomSensor

        configureMotors();
    }

    private void configureMotors() {
        var motorConfigs = new MotorOutputConfigs();

        var slot0 = new Slot0Configs();
        // TODO tune pids

        rightMotor.getConfigurator().apply(slot0);
        rightMotor.getConfigurator().apply(motorConfigs.withInverted(InvertedValue.Clockwise_Positive));
        
        leftMotor.getConfigurator().apply(slot0);
        leftMotor.getConfigurator().apply(motorConfigs.withInverted(InvertedValue.CounterClockwise_Positive));
    }
    
    public boolean isAtBottom() {
        return bottomSensor.get();
    }

    public double getHeight() {
        return heightSensor.getAmbientSignal().getValue();
    }

    public Command setHeight(double height) { return Commands.none(); }
}
