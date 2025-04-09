package frc.robot.subsystems.coralarmgripper;

import static frc.robot.subsystems.coralarmgripper.CoralArmGripperConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralArmGripperSubsystem extends SubsystemBase {
    private static CoralArmGripperSubsystem instance;

    private static final TalonFX gripper_motor = new TalonFX(gripperMotorID, "canivore");

    private CoralArmGripperSubsystem() {
        configureMotors();
    }

    public static synchronized CoralArmGripperSubsystem getInstance() {
        if (instance == null) {
            instance = new CoralArmGripperSubsystem();
        }

        return instance;
    }

    private void configureMotors() {
        var gripper_cfg = new TalonFXConfiguration();
        gripper_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        gripper_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        gripper_cfg.CurrentLimits.SupplyCurrentLimit = gripperMotorCurrentLimit;
    }

    // Gripper
    public Command setVoltage(double voltage) {
        return run(() -> gripper_motor.setControl(new VoltageOut(voltage)));
    }
}