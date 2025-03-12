package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignPose {
    private static final ProfiledPIDController controller_x = new ProfiledPIDController(3, 0, 0,
        new TrapezoidProfile.Constraints(3, 2));
    private static final ProfiledPIDController controller_y = new ProfiledPIDController(3, 0, 0,
        new TrapezoidProfile.Constraints(3, 2));
    private static final ProfiledPIDController controller_theta = new ProfiledPIDController(2, 0, 0,
        new TrapezoidProfile.Constraints(8, 8));

    private static boolean posesNear(Pose2d pose1, Pose2d pose2) {
        double position_tolerance = 0.5;
        double angle_tolerance = 1.0;

        return MathUtil.isNear(pose1.getX(), pose2.getX(), position_tolerance) &&
               MathUtil.isNear(pose1.getY(), pose2.getY(), position_tolerance) &&
               MathUtil.isNear(pose1.getRotation().getDegrees(), pose2.getRotation().getDegrees(), angle_tolerance);
    }

    public static Command alignPose(
        CommandSwerveDrivetrain drivetrain,
        SwerveRequest.FieldCentric driveRequest,
        Pose2d goalPose
    ) {
        Pose2d curr_pose = drivetrain.getState().Pose;
        return Commands.run(() -> drivetrain.setControl(driveRequest
            .withVelocityX(controller_x.calculate(curr_pose.getX(), goalPose.getX()))
            .withVelocityY(controller_y.calculate(curr_pose.getY(), goalPose.getY()))
            .withRotationalRate(controller_theta.calculate(curr_pose.getRotation().getDegrees(), goalPose.getRotation().getDegrees())))
        )
            .until(() -> posesNear(goalPose, curr_pose))
            .andThen(() -> drivetrain.setControl(driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0))
            );
    }
}
