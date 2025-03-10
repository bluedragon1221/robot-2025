package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Elevator extends SubsystemBase {
    private static Elevator instance;

    private static final TalonFX leader_motor = new TalonFX(leftMotorID, "canivore"); 
    private static final TalonFX follower_motor = new TalonFX(rightMotorID, "canivore");
    private static final MotionMagicVoltage mm_voltage = new MotionMagicVoltage(0).withEnableFOC(true);
    
    private static CANrange canrange = new CANrange(heightSensorID, "canivore");

    private static class TuneableConstants {
        private static double kA = 0.01;
        private static double kG = 0.067;
        private static double kS = 0;
        private static double kP = 60;
        private static double kI = 0;
        private static double kD = 0.5;

        public static void initDashboard() {
            SmartDashboard.putNumber("Elevator/kA", kA);
            SmartDashboard.putNumber("Elevator/kG", kG);
            SmartDashboard.putNumber("Elevator/kS", kS);
            SmartDashboard.putNumber("Elevator/kP", kP);
            SmartDashboard.putNumber("Elevator/kI", kI);
            SmartDashboard.putNumber("Elevator/kD", kD);
        }

        public static boolean updateDashboard() {
            double newKA = SmartDashboard.getNumber("CoralArm/kA", kA);
            double newKG = SmartDashboard.getNumber("CoralArm/kG", kG);
            double newKS = SmartDashboard.getNumber("CoralArm/kS", kS);
            double newKP = SmartDashboard.getNumber("CoralArm/kP", kP);
            double newKI = SmartDashboard.getNumber("CoralArm/kI", kI);
            double newKD = SmartDashboard.getNumber("CoralArm/kD", kD);
            
            // Check if any values have changed
            boolean changed = newKA != kA || newKG != kG|| newKS != kS ||
                newKP != kP || newKI != kI || newKD != kD;
            
            // Update stored values
            kA = newKA;
            kG = newKG;
            kS = newKS;
            kP = newKP;
            kI = newKI;
            kD = newKD;
            
            return changed;
        }
    }

    private Elevator() {
        TuneableConstants.initDashboard();

        configureMotors();
    }

    public static synchronized Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private void configureMotors() {
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Encoder
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        cfg.Feedback.SensorToMechanismRatio = motorGearRatio;

        // MotionMagic
        cfg.MotionMagic.MotionMagicAcceleration = motorMaxAcceleration;
        cfg.MotionMagic.MotionMagicCruiseVelocity = motorCruiseVelocity;

        // PID + motionmagic constants
        cfg.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        cfg.Slot0.kA = TuneableConstants.kA;
        cfg.Slot0.kG = TuneableConstants.kG;
        cfg.Slot0.kS = TuneableConstants.kS;
        cfg.Slot0.kP = TuneableConstants.kP;
        cfg.Slot0.kI = TuneableConstants.kI;
        cfg.Slot0.kD = TuneableConstants.kD;

        leader_motor.getConfigurator().apply(cfg);
        follower_motor.getConfigurator().apply(cfg);

        follower_motor.setControl(new Follower(leftMotorID, false));
    }

    public double getHeight() {
        return canrange.getDistance().getValueAsDouble() - canrangeOffset; // subtracts canrangeOffset to get the "actual" position
    }

    public double getRotations() {
        return leader_motor.getPosition().getValueAsDouble();
    }

    public Trigger isAtHeight(double goalHeight) {
        return new Trigger(() -> MathUtil.isNear(goalHeight, getHeight(), heightTolerance));
    }

    private static double translateHeightToRotations(double goalHeight) {
        return goalHeight / (2 * Math.PI * sprocketRadius);
    }

    private static double translateRotationsToHeight(double goalAngle) {
        return (2 * Math.PI * sprocketRadius) / goalAngle;
    }

    public Command setHeight(double goalHeight) {
        return Commands.run(() -> {
            leader_motor.setPosition(translateHeightToRotations(getHeight()));

            leader_motor.setControl(mm_voltage.withPosition(translateHeightToRotations(goalHeight)));
        });
    }

    public Command stopElevator() {
        return Commands.runOnce(() -> {
            leader_motor.setControl(new VoltageOut(0));
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("CANrange Reading", String.format("%.9f", getHeight() + canrangeOffset));
        SmartDashboard.putString("Elevator Reported Height (meters)", String.format("%.9f", translateRotationsToHeight(getRotations())));

        boolean changed = TuneableConstants.updateDashboard();
        if (changed) {
            configureMotors();
            System.out.println("Reconfigured Elevator");
        }
    }
}
