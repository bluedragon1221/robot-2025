package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignReef extends Command {
    private static final TrapezoidProfile.Constraints x_constraints = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints y_constraints = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints theta_constraints = new TrapezoidProfile.Constraints(8, 8);

    private final ProfiledPIDController controller_x;
    private final ProfiledPIDController controller_y;
    private final ProfiledPIDController controller_theta;

    private final Pose2d goal_pose;

    private final SwerveRequest.FieldCentric drive_request;
    private final CommandSwerveDrivetrain drivetrain;

    public AlignReef(CommandSwerveDrivetrain driveTrain, SwerveRequest.FieldCentric driveRequest, Pose2d goalPose) {
        drive_request = driveRequest;
        drivetrain = driveTrain;
        goal_pose = goalPose;

        controller_x = new ProfiledPIDController(3, 0, 0, x_constraints);
        controller_y = new ProfiledPIDController(3, 0, 0, y_constraints);
        controller_theta = new ProfiledPIDController(2, 0, 0, theta_constraints);
    }

    private static boolean posesNear(Pose2d pose1, Pose2d pose2) {
        double position_tolerance = 0.5;
        double angle_tolerance = 1.0;

        return (Math.abs(pose1.getX() - pose2.getX()) < position_tolerance) &&
               (Math.abs(pose1.getY() - pose2.getY()) < position_tolerance) &&
               (Math.abs(pose1.getRotation().getDegrees() - pose2.getRotation().getDegrees()) < angle_tolerance);
    }

    @Override
    public void initialize() {
        controller_x.reset(drivetrain.getState().Pose.getX());
        controller_y.reset(drivetrain.getState().Pose.getY());
        controller_theta.reset(drivetrain.getState().Pose.getRotation().getDegrees());
    }

    @Override
    public void execute() {
        Pose2d curr_pose = drivetrain.getState().Pose;
        drivetrain.setControl(
            drive_request
                .withVelocityX(controller_x.calculate(curr_pose.getX(), goal_pose.getX()))
                .withVelocityY(controller_y.calculate(curr_pose.getY(), goal_pose.getY()))
                .withRotationalRate(controller_theta.calculate(curr_pose.getRotation().getDegrees(), goal_pose.getRotation().getDegrees()))
        );
    }

    @Override
    public boolean isFinished() {
        Pose2d curr_pose = drivetrain.getState().Pose;
        return posesNear(goal_pose, curr_pose);
    }

    @Override
    public void end(boolean interupted) {
        drivetrain.setControl(
            drive_request
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }
}
