// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.control.Launchpad;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.supersystems.ElevatorSupersystem;
import frc.robot.supersystems.ElevatorSupersystem.CoralLayer;
import frc.robot.Constants.Preset;

public class RobotContainer {
    // initialize subsystems
    Elevator elevator = Elevator.getInstance();
    ElevatorSupersystem supersystem = ElevatorSupersystem.getInstance();
    CoralArm coral_arm = CoralArm.getInstance();
    Climber climber = Climber.getInstance();

    Launchpad launchpad = new Launchpad(1, 2, 3, new Color8Bit(255, 255, 255));

    private LinearVelocity max_speed = TunerConstants.kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    private AngularVelocity max_angular_rate = RotationsPerSecond.of(0.75); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(max_speed.in(MetersPerSecond) * 0.1)
            .withRotationalDeadband(max_angular_rate.in(RadiansPerSecond) * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(max_speed.in(MetersPerSecond));

    private final CommandXboxController controller = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Choreo stuff
    private final AutoFactory auto_factory;
    private final AutoRoutines auto_routines;
    private final AutoChooser auto_chooser = new AutoChooser();

    public RobotContainer() {
        // Add Autos
        auto_factory = drivetrain.createAutoFactory()
            .bind("set_l4_height", supersystem.coralPrepareElevator(CoralLayer.L4))
            .bind("score_l4", supersystem.coralScoreCoral(CoralLayer.L4))
            .bind("prepare_accept_coral", supersystem.intakeSetupIntake())
            .bind("accept_coral", supersystem.intakeLoadIntake());

        auto_routines = new AutoRoutines(auto_factory);
        auto_chooser.addRoutine("Center Cage 2L4", auto_routines::CenterCage2l4);

        SmartDashboard.putData("Auto Chooser", auto_chooser);

        configureBindings();
    }

    private boolean manual_turtle_mode = false;
    private double turtle_mode = 1.0;

    public void updateTurtleMode() {
        double curr_height = elevator.getHeight();
        turtle_mode = ((curr_height >= Preset.ScoreL3.getHeight()) || manual_turtle_mode) ? 0.357 : 1;
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        
         // Drivetrain will execute this command periodically
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(
                () -> drive.withVelocityX(-controller.getLeftY() * max_speed.in(MetersPerSecond) * turtle_mode) // Drive forward with negative Y (forward)
                        .withVelocityY(-controller.getLeftX() * max_speed.in(MetersPerSecond) * turtle_mode) // Drive left with negative X (left)
                        .withRotationalRate(-controller.getRightX() * max_angular_rate.in(RotationsPerSecond) * turtle_mode) // Drive counterclockwise with negative X (left)
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

        launchpad.getButton(9, 1).onTrue(elevator.runOnce(() -> elevator.setHeight(Preset.ScoreL4.getHeight())));
        launchpad.getButton(9, 2).onTrue(elevator.runOnce(() -> elevator.setHeight(Preset.ScoreL3.getHeight())));
        launchpad.getButton(9, 3).onTrue(elevator.runOnce(() -> elevator.setHeight(Preset.ScoreL2.getHeight())));
        launchpad.getButton(9, 4).onTrue(elevator.runOnce(() -> elevator.setHeight(Preset.ScoreL1.getHeight())));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return auto_chooser.selectedCommand();
    }
}
