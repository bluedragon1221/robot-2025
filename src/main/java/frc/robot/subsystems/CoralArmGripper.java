package frc.robot.subsystems;

import static frc.robot.Constants.CoralArmGripperConstants.beamBreakChannelDIO;
import static frc.robot.Constants.CoralArmGripperConstants.gripperMotorCurrentLimit;
import static frc.robot.Constants.CoralArmGripperConstants.gripperMotorID;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CoralArmGripper extends SubsystemBase {
    private static CoralArmGripper instance;

    private final SparkMax gripper_motor = new SparkMax(gripperMotorID, MotorType.kBrushless);
    private final DigitalInput beam_break = new DigitalInput(beamBreakChannelDIO); 

    private final Trigger beam_broken = new Trigger(() -> beam_break.get());

    private CoralArmGripper() {
        configureMotors();

        SmartDashboard.putNumber("Set Gripper Voltage", 0);
    }

    public static synchronized CoralArmGripper getInstance() {
        if (instance == null) {
            instance = new CoralArmGripper();
        }

        return instance;
    }

    private void configureMotors() {
        var gripper_cfg = new SparkMaxConfig();
        gripper_cfg.idleMode(IdleMode.kBrake);
        gripper_cfg.smartCurrentLimit((int) gripperMotorCurrentLimit);
        gripper_motor.configure(gripper_cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Trigger hasCoral() {
        return beam_broken;
    }

    // Gripper
    public Command setGripperVoltage(double voltage) {
        return run(() -> gripper_motor.setVoltage(voltage));
    }

    public Command setVoltageFromDashboard() {
        return run(() -> {
            gripper_motor.setVoltage(SmartDashboard.getNumber("Set Gripper Voltage", 0));
        });
    }
}