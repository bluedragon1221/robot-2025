package frc.robot.subsystems;

import static frc.robot.Constants.AlgaeArmPivotConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArmPivot extends SubsystemBase {
    private static AlgaeArmPivot instance;

    private static final TalonFX pivotMotor = new TalonFX(pivotMotorID, "canivore");
    private static final MotionMagicVoltage pivot_position_voltage = new MotionMagicVoltage(0).withEnableFOC(true);

    AlgaeArmPivot() {
        configureMotors();
    }

    public static synchronized AlgaeArmPivot getInstance() {
        if (instance == null) {
            instance = new AlgaeArmPivot();
        }

        return instance;
    }

    private void configureMotors() { 
        var pivot_cfg = new TalonFXConfiguration();
    
        // Use internal sensor as feedback source
        pivot_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivot_cfg.Feedback.SensorToMechanismRatio = pivotMotorGearRatio;
    
        // MotionMagic
        pivot_cfg.MotionMagic.MotionMagicAcceleration = pivotMotorAcceleration;
        pivot_cfg.MotionMagic.MotionMagicCruiseVelocity = pivotMotorCruiseVelocity;
    
        // PID
        pivot_cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivot_cfg.Slot0.kG = 0.067;
        pivot_cfg.Slot0.kP = 60;
        pivot_cfg.Slot0.kI = 0;
        pivot_cfg.Slot0.kD = 0.5;
        
        pivotMotor.getConfigurator().apply(pivot_cfg);
    }

    public Command setPivotAngle(double goalAngle) {
        return run(() -> pivotMotor.setControl(
            pivot_position_voltage.withPosition(goalAngle)
        ));
    }
}