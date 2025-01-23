package frc.robot.subsystems;

import static frc.robot.Constants.CoralArmConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralArm extends SubsystemBase {
    private static TalonFX pivotMotor;
    private static SparkMax gripperMotor;

    public CoralArm() {
        pivotMotor = new TalonFX(pivotMotorID, "canbus");
        gripperMotor = new SparkMax(gripperMotorID, MotorType.kBrushless);

        configureMotors();
    }

    private void configureMotors() {
        var pivotMotorConfigs = new TalonFXConfiguration();
        // TODO tune pivot motor
        // TODO user motionmagic

        pivotMotor.getConfigurator().apply(pivotMotorConfigs);
    }

    public double getPivotAngle() { }
    public Command setPivotAngle() { return Commands.none(); }

    public Command setGripperSpeed(double speed) { return Commands.none(); }
    public Command seqStopGripper() { return setGripperSpeed(0); }
}
