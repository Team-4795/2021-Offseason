// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnToGoal;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VisionModule;

public class Autonomous {
  private Drivebase drivebase;
  private Shooter shooter;
  private Indexer indexer;
  private Intake intake;
  private VisionModule visionModule;

  public Autonomous(Drivebase drivebase, Shooter shooter, Indexer indexer, Intake intake, VisionModule visionModule) {
    this.drivebase = drivebase;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    this.visionModule = visionModule;
  }

  public Command shootAndDriveBack() {
    return new ParallelCommandGroup(
      new TurnToGoal(drivebase, visionModule),
      new Shoot(shooter, visionModule, true),
      new SequentialCommandGroup(
        new InstantCommand(intake::extend, intake),
        new WaitCommand(1.5),
        new InstantCommand(() -> indexer.setIndexerSpeed(0.6, 0.75), indexer),
        new WaitCommand(0.5),
        new InstantCommand(() -> intake.setIntakeSpeed(0.4), intake),
        new WaitCommand(3),
        new FunctionalCommand(
          drivebase::resetEncoders,
          () -> drivebase.curvatureDrive(-0.25, 0, false),
          interrupted -> drivebase.curvatureDrive(0, 0, false),
          () -> drivebase.getLeftEncoder() / 10.0 * DrivebaseConstants.wheelDiameterMeters * Math.PI <= -1.5,
          drivebase
        )
      )
    );
  }

  public Command intakeAndShoot() {
    return new SequentialCommandGroup(
      new InstantCommand(intake::extend, intake),
      new WaitCommand(1.5),
      new InstantCommand(() -> intake.setIntakeSpeed(0.6), intake),
      new FunctionalCommand(
        drivebase::resetEncoders,
        () -> drivebase.curvatureDrive(-0.25, 0, false),
        interrupted -> drivebase.curvatureDrive(0, 0, false),
        () -> drivebase.getLeftEncoder() / 10.0 * DrivebaseConstants.wheelDiameterMeters * Math.PI <= -3,
        drivebase
      ),
      new InstantCommand(() -> intake.setIntakeSpeed(0), intake),
      new ParallelCommandGroup(
        new TurnToGoal(drivebase, visionModule),
        new Shoot(shooter, visionModule, true),
        new SequentialCommandGroup(
          new WaitCommand(1.5),
          new InstantCommand(() -> indexer.setIndexerSpeed(0.6, 0.75), indexer),
          new WaitCommand(0.5),
          new InstantCommand(() -> intake.setIntakeSpeed(0.4), intake)
        )
      )
    );
  }

  public Command createRamsete() {
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
}
