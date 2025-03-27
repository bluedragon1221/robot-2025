package frc.robot.supersystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.L1Gripper;
import frc.robot.subsystems.L1Gripper.GripperVoltage;
import frc.robot.subsystems.L1Pivot;
import frc.robot.subsystems.L1Pivot.PivotAngle;;

public class L1Supersystem {
    private static L1Supersystem instance;

    public static final L1Gripper l1_gripper = L1Gripper.getInstance();
    public static final L1Pivot l1_pivot = L1Pivot.getInstance();

    public Command setState(double pivot_angle, double gripper_voltage) {
        return Commands.parallel(
            l1_gripper.setGripperVoltage(gripper_voltage),
            l1_pivot.setPivotAngle(pivot_angle));
    }

    public Command setStatePivot(double pivot_angle) {
        return Commands.parallel(
                l1_pivot.setPivotAngle(pivot_angle));
    }

    public Command setStateGripper(double gripper_voltage) {
        return Commands.parallel(
                l1_gripper.setGripperVoltage(gripper_voltage));
    }

    public static synchronized L1Supersystem getInstance() {
        if (instance == null) {
            instance = new L1Supersystem();
        }

        return instance;
    }

    public Command deployIntake() {
        return setState(PivotAngle.intake, GripperVoltage.intake);
    }

    public Command returnIntake() {
        return setState(PivotAngle.storage, GripperVoltage.zero);
    }

    public Command scoreL1() {
        return setStatePivot(PivotAngle.score)
            .until(l1_pivot.isAtAngle(PivotAngle.score))
            .andThen(setState(PivotAngle.score, GripperVoltage.score));
    }

}
