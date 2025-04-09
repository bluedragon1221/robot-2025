package frc.robot.subsystems.coralarmpivot;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.elevator.ElevatorPreset;

public class CoralArmPivotSubsystemSysID extends CoralArmPivotSubsystem {
    private static CoralArmPivotSubsystemSysID instance;

    private final Trigger atMax = new Trigger(() -> MathUtil.isNear(pivot_angle.get(), ElevatorPreset.max, 0.01));
    private final Trigger atMin = new Trigger(() -> MathUtil.isNear(pivot_angle.get(), ElevatorPreset.min, 0.01));

    private static final VoltageOut sysid_voltage = new VoltageOut(0);

    public static synchronized CoralArmPivotSubsystemSysID getInstance() {
        if (instance == null) {
            instance = new CoralArmPivotSubsystemSysID();
        }

        return instance;
    }

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
        new SysIdRoutine.Config(
            Volts.per(Second).of(0.5),
            Volts.of(2),
            null,
            state -> SignalLogger.writeString("state", state.toString()) // how to log state
        ),
        new SysIdRoutine.Mechanism(
            output -> pivot_motor.setControl(sysid_voltage.withOutput(output)), // how to apply voltage to motor
            null,
            this
        )
    );

    public Command runSysIdCommand() {
        return Commands.sequence(
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(atMax),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(atMin),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(atMax),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(atMin),
            Commands.print("DONE")
        );
    }
}
