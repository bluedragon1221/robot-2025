// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Preset;
import frc.robot.control.Launchpad;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralArmGripper;
import frc.robot.subsystems.CoralArmPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.supersystems.ElevatorSupersystem;

public class RobotContainer {
    // initialize subsystems
    Elevator elevator = Elevator.getInstance();
    ElevatorSupersystem supersystem = ElevatorSupersystem.getInstance();
    CoralArmPivot coral_arm_pivot = CoralArmPivot.getInstance();
    CoralArmGripper coral_arm_gripper = CoralArmGripper.getInstance();
    Climber climber = Climber.getInstance();

    Launchpad launchpad = new Launchpad(1, 2, 3, new Color8Bit(255, 255, 255));

    private double max_speed = TunerConstants.kSpeedAt12Volts.magnitude(); // kSpeedAt12Volts desired top speed
                                                                                               // desired top speed
    private double max_angular_rate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(max_speed * 0.1)
            .withRotationalDeadband(max_angular_rate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric robot_centric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(max_speed);
    private final CommandXboxController controller = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Choreo stuff
    private final AutoFactory auto_factory;
    private final AutoRoutines auto_routines;
    private final AutoChooser auto_chooser = new AutoChooser();

    public RobotContainer() {
        // Add Autos
        auto_factory = drivetrain.createAutoFactory()
        .bind("storage_position", supersystem.storagePosition())
        .bind("intake_prepare", supersystem.intakePrepare())
        .bind("intake_load", supersystem.intakeLoad())
        .bind("intake_post", supersystem.intakePost())
        .bind("coral_prepare_l4", supersystem.coralPrepareL4())
        .bind("coral_score_l4", supersystem.coralScoreL4());

        auto_routines = new AutoRoutines(auto_factory);
        auto_chooser.addRoutine("Center Cage 2L4", auto_routines::CenterCage2l4);

        SmartDashboard.putData("Auto Chooser", auto_chooser);

        configureBindings();
    }

    private boolean manual_turtle_mode = false;

    private final double turtle_mode = 0.25;
    private Trigger turtle_trigger = new Trigger(() -> ((elevator.getHeight() >= Preset.IntakeCatch.getHeight()) && (elevator.getHeight() <= Preset.ScoreL3.getHeight())));

    private final double slower_turtle_mode = 0.15;
    private Trigger slower_turtle_trigger = new Trigger(() -> ((elevator.getHeight() >= Preset.ScoreL3.getHeight()) || manual_turtle_mode));

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        // Drivetrain will execute this command periodically
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(
                () -> drive.withVelocityX(-controller.getLeftY() * max_speed) // Drive forward with negative Y (forward)
                           .withVelocityY(-controller.getLeftX() * max_speed) // Drive left with negative X (left) Drive counterclockwise with negative X (left)
                           .withRotationalRate(-controller.getRightX() * max_angular_rate) // Drive counterclockwise with negative X (left)

            ));

        turtle_trigger.whileTrue(drivetrain.applyRequest(
            () -> drive.withVelocityX(-controller.getLeftY() * max_speed * turtle_mode) // Drive forward with negative Y (forward)
                       .withVelocityY(-controller.getLeftX() * max_speed * turtle_mode) // Drive left with negative X (left)
                       .withRotationalRate(-controller.getRightX() * max_angular_rate * turtle_mode) // Drive counterclockwise with negative X (left)
        ));

        slower_turtle_trigger.whileTrue(drivetrain.applyRequest(
                () -> drive.withVelocityX(-controller.getLeftY() * max_speed * slower_turtle_mode) // Drive forward with negative Y (forward)
                           .withVelocityY(-controller.getLeftX() * max_speed * slower_turtle_mode) // Drive left with negative X (left)
                           .withRotationalRate(-controller.getRightX() * max_angular_rate * slower_turtle_mode) // Drive counterclockwise with negative X (left)
            ));

        controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        controller.povLeft().whileTrue(
                drivetrain.applyRequest(() -> robot_centric.withVelocityX(0).withVelocityY(max_speed * slower_turtle_mode)));
        controller.povRight().whileTrue(
                drivetrain.applyRequest(() -> robot_centric.withVelocityX(0).withVelocityY(-max_speed * slower_turtle_mode)));
        controller.povUp().whileTrue(
                drivetrain.applyRequest(() -> robot_centric.withVelocityX(max_speed * slower_turtle_mode).withVelocityY(0)));
        controller.povDown().whileTrue(
                drivetrain.applyRequest(() -> robot_centric.withVelocityX(-max_speed * slower_turtle_mode).withVelocityY(0)));

        // Elevator/coral arm controls
        launchpad.getButton(8, 1).onTrue(supersystem.coralPrepareL1());
        launchpad.getButton(8, 2).onTrue(supersystem.coralPrepareL2());
        launchpad.getButton(8, 3).onTrue(supersystem.coralPrepareL3());
        launchpad.getButton(8, 4).onTrue(supersystem.coralPrepareL4());
        
        launchpad.getButton(7, 1).onTrue(supersystem.coralScoreL1());
        launchpad.getButton(7, 2).onTrue(supersystem.coralScoreL2());
        launchpad.getButton(7, 3).onTrue(supersystem.coralScoreL3());
        launchpad.getButton(7, 4).onTrue(supersystem.coralScoreL4());

        launchpad.getButton(8, 5).onTrue(supersystem.intakePrepare());
        launchpad.getButton(7, 5).onTrue(supersystem.intakeLoad());
        launchpad.getButton(6, 5).onTrue(supersystem.intakePost());

        launchpad.getButton(8, 6).onTrue(supersystem.extractionPrepareHigh());
        launchpad.getButton(7, 6).onTrue(supersystem.extractionExtractHigh());

        launchpad.getButton(8, 7).onTrue(supersystem.extractionPrepareLow());
        launchpad.getButton(7, 7).onTrue(supersystem.extractionExtractLow());

        launchpad.getButton(8, 8).onTrue(supersystem.extractionStop());

        launchpad.getButton(0, 8).onTrue(supersystem.storagePosition());

        launchpad.getButton(2, 2).onTrue(supersystem.setStateFromDashboard());

        /// manual testing voltage sets
        controller.rightTrigger().onTrue(climber.setVoltage(6)).onFalse(climber.setVoltage(0));
        controller.leftTrigger().onTrue(climber.setVoltage(-6)).onFalse(climber.setVoltage(0));

        // Telemetrize our drive train
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return auto_chooser.selectedCommand();
    }
}
