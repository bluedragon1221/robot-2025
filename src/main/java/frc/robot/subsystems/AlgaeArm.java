package frc.robot.subsystems;

import static frc.robot.Constants.AlgaeArmConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArm extends SubsystemBase {
    private static TalonFX pivotMotor;
    private static SparkMax gripperMotor;
    private static RelativeEncoder gripperEncoder;

    public AlgaeArm() {
        pivotMotor = new TalonFX(pivotMotorID, "canivore");
        gripperMotor = new SparkMax(gripperMotorID, MotorType.kBrushless);
        gripperEncoder = gripperMotor.getEncoder();

        configureMotors();
    }

    private void configureMotors() {
        var pivotMotorConfigs = new TalonFXConfiguration();
        // TODO tune pivot motor
        // TODO use motionmagic

        pivotMotor.getConfigurator().apply(pivotMotorConfigs);
    }

    public double getPivotAngle() { }
    public void setPivotAngle(double angle) {}

    public Command setGripperSpeed(double speed) { return Commands.none(); }
    public Command seqStopGripper() { return setGripperSpeed(0); }

    public Command seqDeployIntake() { return Commands.none(); }
    public Command seqRetreatIntake() { return Commands.none(); }
}
