package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevatorSubsystemSysID extends ElevatorSubsystem {
    private static ElevatorSubsystemSysID instance;
    
    private final Trigger atMax = new Trigger(() -> MathUtil.isNear(elevator_height.get(), ElevatorPreset.max, 0.0254));
    private final Trigger atMin = new Trigger(() -> MathUtil.isNear(elevator_height.get(), ElevatorPreset.min, 0.0254));

    private static final VoltageOut sysid_voltage = new VoltageOut(0);

    public static synchronized ElevatorSubsystemSysID getInstance() {
        if (instance == null) {
            instance = new ElevatorSubsystemSysID();
        }

        return instance;
    }

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(4),
            null,
            state -> SignalLogger.writeString("state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                leader_motor.setControl(sysid_voltage.withOutput(output));
                follower_motor.setControl(follow);
            },
            null,
            this
        )
    );

    public Command runSysICommand() {
        return Commands.sequence(
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(atMax),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(atMin),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(atMax),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(atMin),
            Commands.print("DONE")
        );
    }

}
