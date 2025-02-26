package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.AlgaeArmConstants.gripperMotorID;
import static frc.robot.Constants.AlgaeArmConstants.pivotMotorID;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArm extends SubsystemBase {
    private static AlgaeArm instance;

    private static TalonFX pivotMotor;
    final PositionVoltage position_voltage = new PositionVoltage(0);

    private static SparkMax gripperMotor;

    private AlgaeArm() {
        pivotMotor = new TalonFX(pivotMotorID, "canivore");
        gripperMotor = new SparkMax(gripperMotorID, MotorType.kBrushless);

        configureMotors();
    }

    public static synchronized AlgaeArm getInstance() {
        if (instance == null) {
            instance = new AlgaeArm();
        }

        return instance;
    }

    private void configureMotors() {
        var cfg = new TalonFXConfiguration();
        cfg.Slot0.kG = 0.067;
        cfg.Slot0.kP = 30;
        cfg.Slot0.kI = 0;
        cfg.Slot0.kD = 0.3;

        pivotMotor.getConfigurator().apply(cfg);
    }

    // Gripper
    private Command setGripperVoltage(Voltage voltage) {
        return run(() -> gripperMotor.setVoltage(voltage.magnitude()));
    }

    // Pivot
    private Command setPivotAngle(Rotation2d goalAngle) {
        return run(() -> pivotMotor.setControl(
                position_voltage.withPosition(goalAngle.getRotations())
            )
        );
    }

    // sequences
    public Command deployArm() {
        return Commands.parallel(
            setPivotAngle(Rotation2d.fromDegrees(35)),
            setGripperVoltage(Volts.of(40))
        );
    }

    public Command retreatArm() {
        return Commands.parallel(
            setPivotAngle(Rotation2d.fromDegrees(-10)),
            setGripperVoltage(Volts.of(0))
        );
    }
}
