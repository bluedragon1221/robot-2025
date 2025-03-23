package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.L1GripperConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class L1Gripper extends SubsystemBase {
    private static L1Gripper instance;

    private static final SparkMax gripperMotor = new SparkMax(gripperMotorID, MotorType.kBrushless);

    private L1Gripper() {
        configureMotors();
    }

    public static synchronized L1Gripper getInstance() {
        if (instance == null) {
            instance = new L1Gripper();
        }

        return instance;
    }

    private void configureMotors() {
        var gripper_cfg = new SparkMaxConfig();
        gripper_cfg.smartCurrentLimit(gripperMotorCurrentLimit);
        gripper_cfg.openLoopRampRate(gripperMotorRampRate);
        gripperMotor.configure(gripper_cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command setGripperVoltage(Voltage voltage) {
        return run(() -> gripperMotor.setVoltage(voltage.in(Volts)));
    }
}