// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.commands.EastDrive;
import frc.robot.commands.ZeroHood;
import frc.robot.commands.TurnToGoal;
import frc.robot.commands.Shoot;
import frc.robot.commands.Index;
import frc.robot.commands.Unjam;
import frc.robot.commands.RunTests;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.AutoConstants;

import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class RobotContainer {
  private final Drivebase drivebase = new Drivebase();

  private final Shooter shooter = new Shooter();

  private final Indexer indexer = new Indexer();

  private final Intake intake = new Intake();

  private final Joystick controller = new Joystick(0);

  public RobotContainer() {
    drivebase.setDefaultCommand(new EastDrive(
      drivebase,
      () -> applyDeadband(-controller.getRawAxis(ControllerConstants.LEFT_JOYSTICK)),
      () -> applyDeadband(-controller.getRawAxis(ControllerConstants.RIGHT_JOYSTICK)),
      () -> applyDeadband(controller.getRawAxis(ControllerConstants.RIGHT_TRIGGER))
    ));
    indexer.setDefaultCommand(new Unjam(indexer));
    shooter.setDefaultCommand(new ZeroHood(shooter));
    
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    JoystickButton leftBumper = new JoystickButton(controller, ControllerConstants.LEFT_BUMPER);
    JoystickButton rightBumper = new JoystickButton(controller, ControllerConstants.RIGHT_BUMPER);
    JoystickButton leftTrigger = new JoystickButton(controller, ControllerConstants.LEFT_TRIGGER);
    JoystickButton aButton = new JoystickButton(controller, ControllerConstants.A_BUTTON);
    JoystickButton bButton = new JoystickButton(controller, ControllerConstants.B_BUTTON);

    leftBumper.whileHeld(new ParallelCommandGroup(
      new TurnToGoal(drivebase),
      new Shoot(shooter),
      new SequentialCommandGroup(new WaitCommand(1), new Index(indexer))
    ));
    rightBumper.whenPressed(new InstantCommand(() -> drivebase.reverse()));
    leftTrigger.whileHeld(new InstantCommand(() -> intake.setIntakeSpeed(Math.max(drivebase.getSpeed(), 0.6))));
    leftTrigger.whenReleased(new InstantCommand(() -> intake.setIntakeSpeed(0)));
    aButton.whenPressed(new InstantCommand(() -> intake.extend()));
    bButton.whenPressed(new InstantCommand(() -> intake.retract()));
  }

  public void setRumble(double rumble) {
    controller.setRumble(RumbleType.kLeftRumble, rumble);
    controller.setRumble(RumbleType.kRightRumble, rumble);
  }
  
  public Command getAutonomousCommand() {
    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(DrivebaseConstants.ksVolts, 
            DrivebaseConstants.kvVoltSecondsPerMeter, 
            DrivebaseConstants.kaVoltSecondsSquaredPerMeter),
            DrivebaseConstants.kDriveKinematics,
          10);

    TrajectoryConfig config =
      new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          .setKinematics(DrivebaseConstants.kDriveKinematics)
          .addConstraint(autoVoltageConstraint);
            
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(1, 1)
      ),
      new Pose2d(0, 0, new Rotation2d(0)),
      config);

    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory,
      drivebase::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DrivebaseConstants.ksVolts, DrivebaseConstants.kvVoltSecondsPerMeter, DrivebaseConstants.kaVoltSecondsSquaredPerMeter),
      DrivebaseConstants.kDriveKinematics,
      drivebase::getWheelSpeeds,
      new PIDController(DrivebaseConstants.kPDriveVel, 0, 0),
      new PIDController(DrivebaseConstants.kPDriveVel, 0, 0),
      drivebase::tankDriveVolts,
      drivebase);

    drivebase.resetOdometry(trajectory.getInitialPose());

    return new InstantCommand(() -> drivebase.resetOdometry(trajectory.getInitialPose()), drivebase)
        .andThen(ramseteCommand)
        .andThen(new InstantCommand(() -> drivebase.tankDriveVolts(0, 0), drivebase));
  }

  public Command getTestCommand() {
    return new RunTests(drivebase, intake, indexer, shooter);
  }

  private double applyDeadband(double value) {
    double deadband = ControllerConstants.JOYSTICK_DEADBAND;

    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}
