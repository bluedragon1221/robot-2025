// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// @formatter:off
package frc.robot;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveToPose;
import frc.robot.control.Launchpad;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.coralarmgripper.CoralArmGripperSubsystem;
import frc.robot.subsystems.elevator.ElevatorPreset;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.statusled.StatusLED;
import frc.robot.supersystem.Supersystem;
import frc.robot.util.AllianceFlipUtil;

public class RobotContainer {
    // initialize subsystems
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    StatusLED status_led = StatusLED.getInstance();

    Launchpad launchpad = new Launchpad(1, 2, 3, new Color8Bit(255, 255, 255));

    private double max_speed = TunerConstants.kSpeedAt12Volts.magnitude(); // kSpeedAt12Volts desired top speed
    private double max_angular_rate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(max_speed * 0.1)
            .withRotationalDeadband(max_angular_rate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use closed-loop control for drive motors
    private final SwerveRequest.RobotCentric robot_centric =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(max_speed);
    private final CommandXboxController controller = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Choreo stuff
    private final AutoFactory auto_factory;
    private final AutoRoutines auto_routines;
    private final AutoChooser auto_chooser = new AutoChooser();

    private List<Pose2d> autoalign_lefts;
    private List<Pose2d> autoalign_rights;

    private final Pose2d hps;

    public RobotContainer() {
        if (Robot.isSimulation()) DriverStation.silenceJoystickConnectionWarning(true);
        
        // Targets for autoalign
        autoalign_lefts = AllianceFlipUtil.applyAll(FieldConstants.Reef.lefts);
        autoalign_rights = AllianceFlipUtil.applyAll(FieldConstants.Reef.rights);

        // Add Autos
        auto_factory = drivetrain.createAutoFactory();
        auto_routines = new AutoRoutines(auto_factory);
        auto_chooser.addRoutine("Center 1L4", auto_routines::Center1l4);
        auto_chooser.addRoutine("Drive Forward", auto_routines::DriveForward);
        auto_chooser.addRoutine("Blue Center Cage 2L4", auto_routines::BlueCenterCage2l4);

        SmartDashboard.putData("Auto Chooser", auto_chooser);


        hps = AllianceFlipUtil.apply(new Pose2d(new Translation2d(1.1446170806884766, 0.9114338755607605), Rotation2d.fromRadians(-2.203650142759433)));
        
        configureBindings();
    }

    private final double turtle_mode = 0.15;
    private final double slower_turtle_mode = 0.035;

    private Trigger turtle_trigger = elevator.isAboveHeight(ElevatorPreset.l2).and(DriverStation::isTeleopEnabled);

    // Auto align bindings
    private Supplier<Pose2d> nearestLeftCoral() {
        return () ->
            drivetrain.getState().Pose.nearest(autoalign_lefts);
    }

    private Supplier<Pose2d> nearestRightCoral() {
        return () ->
            drivetrain.getState().Pose.nearest(autoalign_rights);
    }

    public void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        // Drivetrain will execute this command periodically
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive
                    .withVelocityX(-controller.getLeftY() * max_speed) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getLeftX() * max_speed) // Drive left with negative X (left) Drive counterclockwise with negative X (left)
                    .withRotationalRate(-controller.getRightX() * max_angular_rate) // Drive counterclockwise with negative X (left)
        ));

        turtle_trigger.whileTrue(
            drivetrain.applyRequest(() ->
                drive
                    .withVelocityX(-controller.getLeftY() * max_speed * turtle_mode) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getLeftX() * max_speed * turtle_mode) // Drive left with negative X (left)
                    .withRotationalRate(-controller.getRightX() * max_angular_rate * turtle_mode) // Drive counterclockwise with negative X (left)
        ));

        controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        // reset the field-centric heading on left bumper press
        controller.leftBumper().and(controller.start()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        controller.povLeft().whileTrue(
            drivetrain.applyRequest(() ->
                robot_centric
                    .withVelocityX(0)
                    .withVelocityY(max_speed * slower_turtle_mode)
        ));

        controller.povRight().whileTrue(
            drivetrain.applyRequest(() ->
                robot_centric
                    .withVelocityX(0)
                    .withVelocityY(-max_speed * slower_turtle_mode)
        ));

        controller.povUp().whileTrue(
            drivetrain.applyRequest(() ->
                robot_centric
                    .withVelocityX(max_speed * slower_turtle_mode)
                    .withVelocityY(0)
        ));

        controller.povDown().whileTrue(
            drivetrain.applyRequest(() ->
                robot_centric
                    .withVelocityX(-max_speed * slower_turtle_mode)
                    .withVelocityY(0)
        ));

        DriveToPose left_go_to_pose = new DriveToPose(drivetrain, nearestLeftCoral());
        DriveToPose right_go_to_pose = new DriveToPose(drivetrain, nearestRightCoral());
        DriveToPose hps_go_to_pose = new DriveToPose(drivetrain, () -> hps);

        (new Trigger(() -> left_go_to_pose.atGoal() && right_go_to_pose.atGoal())).whileTrue(
            status_led.flashColor(
                new Color(0, 255, 0),
                new Color(255, 255, 0),
                0.25
        ));

        // left/right reef align
        launchpad.getButton(2, 0).or(controller.leftBumper()).whileTrue(left_go_to_pose);
        launchpad.getButton(3, 0).or(controller.rightBumper()).whileTrue(right_go_to_pose);

        launchpad.getButton(1, 0).or(controller.a()).whileTrue(hps_go_to_pose);

        // Elevator/coral arm controls
        launchpad.getButton(8, 1).onTrue(Supersystem.coral.prepareL1());
        launchpad.getButton(8, 2).onTrue(Supersystem.coral.prepareL2());
        launchpad.getButton(8, 3).onTrue(Supersystem.coral.prepareL3());
        launchpad.getButton(8, 4).onTrue(Supersystem.coral.prepareL4());

        // coral scoring
        launchpad.getButton(7, 1).onTrue(Supersystem.coral.scoreL1());
        launchpad.getButton(7, 2).onTrue(Supersystem.coral.scoreL2());
        launchpad.getButton(7, 3).onTrue(Supersystem.coral.scoreL3());
        launchpad.getButton(7, 4).onTrue(Supersystem.coral.scoreL4());

        // intake
        launchpad.getButton(8, 5).onTrue(Supersystem.intake.prepare());
        launchpad.getButton(7, 5).onTrue(Supersystem.intake.load());
        launchpad.getButton(6, 5).onTrue(Supersystem.storage.coral());

        // high extract / low extract
        launchpad.getButton(8, 6).onTrue(Supersystem.extraction.prepareHigh());
        launchpad.getButton(7, 6).onTrue(Supersystem.extraction.extract());

        // low extract
        launchpad.getButton(8, 7).onTrue(Supersystem.extraction.prepareLow());
        launchpad.getButton(7, 7).onTrue(Supersystem.extraction.extract());

        // processor scoring
        launchpad.getButton(6, 7).onTrue(Supersystem.processor.prepare());
        launchpad.getButton(5, 7).onTrue(Supersystem.processor.score());

        // barge scoring
        launchpad.getButton(6, 6).onTrue(Supersystem.barge.prepare());
        launchpad.getButton(5, 6).onTrue(Supersystem.barge.score());

        // storage positions
        launchpad.getButton(0, 7).onTrue(Supersystem.storage.algae());
        launchpad.getButton(0, 8).onTrue(Supersystem.storage.coral());

        // launchpad.getButton(2, 2).onTrue(supersystem.setStateFromDashboard());

        launchpad.getButton(7, 0).onTrue(Commands.runOnce(() -> CoralArmGripperSubsystem.beam_break_override = true))
                        .onFalse(Commands.runOnce(() -> CoralArmGripperSubsystem.beam_break_override = false));

        // Telemetrize our drive train
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return auto_chooser.selectedCommand();
    }
}
