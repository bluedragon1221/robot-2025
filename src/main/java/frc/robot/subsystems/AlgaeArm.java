package frc.robot.subsystems;

import static frc.robot.Constants.AlgaeArmConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArm extends SubsystemBase {
    private static AlgaeArm instance;

    private static TalonFX pivotMotor;
    private static SparkMax gripperMotor;

    private AlgaeArm() {
        pivotMotor = new TalonFX(pivotMotorID, "canivore");
        gripperMotor = new SparkMax(gripperMotorID, MotorType.kBrushless);

        configureMotors();
    }

    public static AlgaeArm getInstance() {
        if (instance == null) {
            instance = new AlgaeArm();
        }

        return instance;
    }

    private void configureMotors() {
        var pivotMotorConfigs = new TalonFXConfiguration();
        // TODO: do these need tuning? mm?

        pivotMotor.getConfigurator().apply(pivotMotorConfigs);
    }

    // Figure out setAngle for algae arm
    public void setPivotAngle(Rotation2d angle) {}

    public Command setGripperVoltage(double voltage) {
        return run(() -> gripperMotor.setVoltage(voltage));
    }

    public Command seqStopGripper() { return setGripperVoltage(0); }

    public Command seqDeployIntake() { return Commands.none(); }
    public Command seqRetreatIntake() { return Commands.none(); }
}
