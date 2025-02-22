package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private static Climber instance;

    private static ClimbState climber_state;
    private static SparkMax motor;

    private enum ClimbState {
        Initial,
        Phase1Active,
        Phase1Complete,
        Phase2Active,
        Phase2Complete,
        Phase3Active,
        ClimbComplete;
    };

    private Climber() {
        setState(ClimbState.Initial);
    
        motor = new SparkMax(climberMotorID, MotorType.kBrushless);
        configureMotors();
    }

    public static synchronized Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }

    private void configureMotors() {
        var cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake);
        cfg.smartCurrentLimit((int)climberMotorCurrentLimit.baseUnitMagnitude());
        cfg.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.5)
            .i(0)
            .d(0.05);

        motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void addRotations(double rotations) {
        double currentPosition = motor.getEncoder().getPosition();
        double targetPosition = (currentPosition + rotations);
        motor.getClosedLoopController().setReference(targetPosition, ControlType.kPosition);
    }

    private void setState(ClimbState state) {
        climber_state = state;
        SmartDashboard.putString("Climber State", state.toString());
    }

    /**
     * Moves the climber arm into position to catch the cage
     * Runs in 0.6 sec
     */
    public Command deployIntake() {
        if (climber_state == ClimbState.Initial) {
            return run(() -> {
                setState(ClimbState.Phase1Active);
                addRotations(50);
                setState(ClimbState.Phase1Complete);
            });
        } else {
            return Commands.none();
        }
    }

    /**
     * Bring the cage into the center of the robot, not supporting the weight of the robot yet.
     * Runs in 1.2 sec
     */
    public Command retrieveCage() {
        if (climber_state == ClimbState.Phase1Complete) {
            return run(() -> {
                setState(ClimbState.Phase2Active);
                addRotations(79);
                setState(ClimbState.Phase2Complete);
            });
        } else {
            return Commands.none();
        }
    }

    /**
     * Complete the climb by pivoting off the cage and supporting the robot from the cage.
     * TODO: Runs at 40% free speed since we're dealing with the full weight of the robot now
     * Runs in 3 sec
     */
    public Command finishClimb() {
        if (climber_state == ClimbState.Phase2Complete) {
            return run(() -> {
                setState(ClimbState.Phase3Active);
                addRotations(81);
                setState(ClimbState.ClimbComplete);
            });
        } else {
            return Commands.none();
        }
    }

    // SmartDashboard: feedback to driver: what state is the robot in?
    //                            camera feed
}
