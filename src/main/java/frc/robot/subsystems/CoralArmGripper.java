package frc.robot.subsystems;

import static frc.robot.Constants.CoralArmGripperConstants.gripperMotorCurrentLimit;
import static frc.robot.Constants.CoralArmGripperConstants.gripperMotorID;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class 
CoralArmGripper extends SubsystemBase {
    private static CoralArmGripper instance;

    private static final TalonFX gripper_motor = new TalonFX(gripperMotorID, "canivore");

    private CoralArmGripper() {
        configureMotors();

        // SmartDashboard.putNumber("Set Gripper Voltage", 0);
    }

    public static synchronized CoralArmGripper getInstance() {
        if (instance == null) {
            instance = new CoralArmGripper();
        }

        return instance;
    }

    private void configureMotors() {
        var gripper_cfg = new TalonFXConfiguration();
        gripper_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       // gripper_cfg.idleMode(IdleMode.kBrake);
       gripper_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
       gripper_cfg.CurrentLimits.SupplyCurrentLimit = gripperMotorCurrentLimit;
       // gripper_cfg.smartCurrentLimit((int) gripperMotorCurrentLimit);
       // neo stuff gripper_motor.configure(gripper_cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Gripper
    public Command setGripperVoltage(double voltage) {
        return run(() -> gripper_motor.setControl(new VoltageOut(voltage)));
    }

    // public Command setVoltageFromDashboard() {
    //     return run(() -> {
    //         gripper_motor.setVoltage(SmartDashboard.getNumber("Set Gripper Voltage", 0));
    //     });
    // }
}