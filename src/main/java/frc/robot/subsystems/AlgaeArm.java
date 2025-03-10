package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.AlgaeArmConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArm extends SubsystemBase {
    private static AlgaeArm instance;

    private static final TalonFX pivotMotor = new TalonFX(pivotMotorID, "canivore");
    private static final MotionMagicVoltage pivot_position_voltage = new MotionMagicVoltage(0).withEnableFOC(true);

    private static final SparkMax gripperMotor = new SparkMax(gripperMotorID, MotorType.kBrushless);

    private static class TuneableConstants {
        private static double kG = 0.067;
        private static double kP = 60;
        private static double kI = 0;
        private static double kD = 0.5;

        public static void initDashboard() {
            SmartDashboard.putNumber("AlgaeArm/kG", kG);
            SmartDashboard.putNumber("AlgaeArm/kP", kP);
            SmartDashboard.putNumber("AlgaeArm/kI", kI);
            SmartDashboard.putNumber("AlgaeArm/kD", kD);
        }
        
        // Method to update constants from SmartDashboard
        public static boolean updateDashboard() {
            double newKG = SmartDashboard.getNumber("AlgaeArm/kG", kG);
            double newKP = SmartDashboard.getNumber("AlgaeArm/kP", kP);
            double newKI = SmartDashboard.getNumber("AlgaeArm/kI", kI);
            double newKD = SmartDashboard.getNumber("AlgaeArm/kD", kD);
            
            // Check if any values have changed
            boolean changed = newKG != kG|| newKP != kP || newKI != kI || newKD != kD;
            
            // Update stored values
            kG = newKG;
            kP = newKP;
            kI = newKI;
            kD = newKD;
            
            return changed;
        }
    }

    private AlgaeArm() {
        configureMotors();

        TuneableConstants.initDashboard();
    }

    public static synchronized AlgaeArm getInstance() {
        if (instance == null) {
            instance = new AlgaeArm();
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
        pivot_cfg.Slot0.kG = TuneableConstants.kG;
        pivot_cfg.Slot0.kP = TuneableConstants.kP;
        pivot_cfg.Slot0.kI = TuneableConstants.kI;
        pivot_cfg.Slot0.kD = TuneableConstants.kD;

        pivotMotor.getConfigurator().apply(pivot_cfg);

        var gripper_cfg = new SparkMaxConfig();
        gripper_cfg.smartCurrentLimit(gripperMotorCurrentLimit);
        gripper_cfg.openLoopRampRate(gripperMotorRampRate);
        gripperMotor.configure(gripper_cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private Command setGripperVoltage(Voltage voltage) {
        return run(() -> gripperMotor.setVoltage(voltage.in(Volts)));
    }

    private Command setPivotAngle(Angle goalAngle) {
        return run(() -> pivotMotor.setControl(
            pivot_position_voltage.withPosition(goalAngle.in(Rotations))
        ));
    }
    
    /**
     * Moves the algae arm out and starts spinning the grippers.
     * Even after we obtain the algae, we need a constant voltage of 3 volts to hold it.
     */
    public Command deployArm() {
        return Commands.parallel(
            setPivotAngle(Degrees.of(45)),
            setGripperVoltage(Volts.of(3))
        );
    }

    /**
     * After we get the algae, move the arm back into the robot so that we can drive around while holding it.
     * Sets the voltage down to 1 volt, which is still enough to keep it.
     */
    public Command returnArm() {
        return Commands.parallel(
            setPivotAngle(Degrees.of(-15)),
            setGripperVoltage(Volts.of(1))
        );
    }

    /**
     * When ready to score the algae, eject it with -3 volts 
     */
    public Command ejectAlgae() {
        return setGripperVoltage(Volts.of(-3))
            .withTimeout(Seconds.of(3))
            .andThen(setGripperVoltage(Volts.of(0)));
    }

    public Command armClimbingMode() {
        return setPivotAngle(Degrees.of(90));
    }

    @Override
    public void periodic() {
        boolean changed = TuneableConstants.updateDashboard();
        if (changed) {
            configureMotors();
            System.out.println("Reconfigured Algae Arm");
        }
    }
}
