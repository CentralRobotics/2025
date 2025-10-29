// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.arm.ArmACW;
import frc.robot.commands.arm.ArmCW;
import frc.robot.commands.claw.ClawRelease;
import frc.robot.commands.claw.ClawGrab;
import frc.robot.commands.elevator.*;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.RumblePatterns;

import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController driverXbox = new CommandXboxController(0);
  public final GenericHID hauteJoystick = new GenericHID(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final ElevatorSubsystem elevatorbase = new ElevatorSubsystem();
  private final ClawSubsystem clawbase = new ClawSubsystem();
  private final ArmSubsystem armbase = new ArmSubsystem();
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String defaultAuto = "testAuto";

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                    () -> driverXbox.getLeftY() * -1,
                                                                    () -> driverXbox.getLeftX() * -1)
                                                                    .withControllerRotationAxis(driverXbox::getRightX)
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */


  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
      .robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                      () -> -driverXbox.getLeftY(),
                                                                      () -> -driverXbox.getLeftX())
                                                                      .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                                      .deadband(OperatorConstants.DEADBAND)
                                                                      .scaleTranslation(0.8)
                                                                      .allianceRelativeControl(true);

  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                                                                      .withControllerHeadingAxis(
                                                                          () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                                                                          () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
                                                                      .headingWhile(true)
                                                                      .translationHeadingOffset(true)
                                                                      .translationHeadingOffset(Rotation2d.fromDegrees(0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("ElevatorUp", new ElevatorUp(elevatorbase));
    autoChooser.addOption(defaultAuto, defaultAuto);
    autoChooser.setDefaultOption(defaultAuto, defaultAuto);
    SmartDashboard.putData("auto", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  @SuppressWarnings("unused")
  private void configureBindings() {

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);
    Command elevatorMoveToUpPosition = new ElevatorUp(elevatorbase);
    Command elevatorReturnToHomePosition = new ElevatorReturnHome(elevatorbase);
    Command armMoveinCockWiseMotion = new ArmCW(armbase);
    Command armMoveinACockWiseMotion = new ArmACW(armbase);
    Command clawGrab = new ClawGrab(clawbase);
    Command clawClose = new ClawRelease(clawbase);
    Command armCW = new ArmCW(armbase);
    Command armACW = new ArmACW(armbase);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(
          () -> target,
          new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
          new ProfiledPIDController(
              5, 0, 0,
              new Constraints(
                  Units.degreesToRadians(360),
                  Units.degreesToRadians(180))));

      driverXbox.start().onTrue(
          Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(
          Commands.runEnd(
              () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
              () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
      // driverXbox.b().whileTrue(
      // drivebase.driveToPose(
      // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
    }


    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());

      // Hold RB to command all module angles to 0° (desired = 0, speed = 0)
      driverXbox.rightBumper().whileTrue(drivebase.centerModulesCommand());
    } else {

      // EVERY OTHER MOD
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());


      // ELEVATOR :0
      new JoystickButton(hauteJoystick, 1).onTrue(elevatorMoveToUpPosition);
      new JoystickButton(hauteJoystick, 2).onTrue(elevatorReturnToHomePosition);
      new JoystickButton(hauteJoystick, 3).onTrue(armMoveinACockWiseMotion);
      new JoystickButton(hauteJoystick, 4).onTrue(armMoveinCockWiseMotion);
      //new JoystickButton(hauteJoystick, 5).whileTrue(ClawGrab);
      driverXbox.y().onTrue(elevatorMoveToUpPosition);
      driverXbox.a().onTrue(
        Commands.sequence(
            Commands.runOnce(drivebase::zeroGyro),
            RumblePatterns.success(driverXbox) 
        )
    );
    driverXbox.y().onTrue(
    elevatorMoveToUpPosition
        .andThen(RumblePatterns.success(driverXbox))
);

driverXbox.b().onTrue(
    elevatorReturnToHomePosition
        .andThen(RumblePatterns.doublePulse(driverXbox)));
    
    


      /*
       * 
       */

      
driverXbox.rightBumper().whileTrue(
    drivebase.centerModulesCommand()
        .andThen(RumblePatterns.doublePulse(driverXbox))
);    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        return drivebase.getAutonomousCommand(autoChooser.getSelected());
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void zeroGyroWithAlliance() {
    drivebase.zeroGyroWithAlliance();
  }
}
