// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionModule;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.commands.EastDrive;
import frc.robot.commands.ZeroHood;
import frc.robot.commands.TurnToGoal;
import frc.robot.commands.Shoot;
import frc.robot.commands.RunTests;
import frc.robot.Constants.ControllerConstants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
  private final Drivebase drivebase = new Drivebase();

  private final Shooter shooter = new Shooter();

  private final Indexer indexer = new Indexer();

  private final Intake intake = new Intake();

  private final VisionModule visionModule = new VisionModule();

  private final Joystick controller = new Joystick(0);

  private final Autonomous autonomous = new Autonomous(drivebase, shooter, indexer, intake, visionModule);

  private final SendableChooser<Command> autonomousSelector = new SendableChooser<>();

  public RobotContainer() {
    drivebase.setDefaultCommand(new EastDrive(
      drivebase,
      () -> applyDeadband(-controller.getRawAxis(ControllerConstants.LEFT_JOYSTICK)),
      () -> applyDeadband(-controller.getRawAxis(ControllerConstants.RIGHT_JOYSTICK)),
      () -> applyDeadband(controller.getRawAxis(ControllerConstants.RIGHT_TRIGGER))
    ));
    indexer.setDefaultCommand(new RunCommand(() -> indexer.setIndexerSpeed(0, -0.25), indexer));
    intake.setDefaultCommand(new RunCommand(() -> intake.setIntakeSpeed(0), intake));
    shooter.setDefaultCommand(new ZeroHood(shooter));

    autonomousSelector.addOption("Shoot and drive back", autonomous.shootAndDriveBack());
    autonomousSelector.addOption("Intake and shoot", autonomous.intakeAndShoot());

    SmartDashboard.putData(autonomousSelector);
    
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    JoystickButton leftBumper = new JoystickButton(controller, ControllerConstants.LEFT_BUMPER);
    JoystickButton rightBumper = new JoystickButton(controller, ControllerConstants.RIGHT_BUMPER);
    Trigger leftTrigger = new Trigger(() -> controller.getRawAxis(2) > ControllerConstants.JOYSTICK_DEADBAND);
    JoystickButton aButton = new JoystickButton(controller, ControllerConstants.A_BUTTON);
    JoystickButton bButton = new JoystickButton(controller, ControllerConstants.B_BUTTON);
    JoystickButton xButton = new JoystickButton(controller, ControllerConstants.X_BUTTON);
    JoystickButton yButton = new JoystickButton(controller, ControllerConstants.Y_BUTTON);
    POVButton up = new POVButton(controller, ControllerConstants.UP);
    POVButton down = new POVButton(controller, ControllerConstants.DOWN);
    POVButton left = new POVButton(controller, ControllerConstants.LEFT);

    leftBumper.whenHeld(new ParallelCommandGroup(
      new TurnToGoal(drivebase, visionModule),
      new Shoot(shooter, visionModule, true),
      new SequentialCommandGroup(
        new InstantCommand(intake::extend, intake),
        new WaitUntilCommand(() -> shooter.getShooterRPM() > 3600 && Math.abs(visionModule.getTargetAngle()) < 1.5),
        new InstantCommand(() -> indexer.setIndexerSpeed(0.6, 0.75), indexer),
        new WaitCommand(0.5),
        new InstantCommand(() -> intake.setIntakeSpeed(0.4), intake)
      )
    ));

    xButton.whenHeld(new ParallelCommandGroup(
      new TurnToGoal(drivebase, visionModule),
      new Shoot(shooter, visionModule, false),
      new SequentialCommandGroup(
        new InstantCommand(intake::extend, intake),
        new WaitUntilCommand(() -> shooter.getShooterRPM() > 3600 && Math.abs(visionModule.getTargetAngle()) < 1.5),
        new InstantCommand(() -> indexer.setIndexerSpeed(0.6, 0.75), indexer),
        new WaitCommand(0.5),
        new InstantCommand(() -> intake.setIntakeSpeed(0.4), intake)
      )
    ));

    rightBumper.whenPressed(drivebase::reverse);

    leftTrigger.whileActiveContinuous(new ParallelCommandGroup(
      new InstantCommand(() -> intake.setIntakeSpeed(Math.max(drivebase.getSpeed(), 0.4)), intake)
    ));

    aButton.whenPressed(intake::extend, intake);
    bButton.whenPressed(intake::retract, intake);

    yButton.whileHeld(new ParallelCommandGroup(
      new InstantCommand(() -> shooter.setHoodAngle(40), shooter),
      new InstantCommand(() -> intake.setIntakeSpeed(-0.4), intake)
    ));

    up.whenPressed(() -> SmartDashboard.putNumber("hood_preset", 0));
    down.whenPressed(() -> SmartDashboard.putNumber("hood_preset", 2));
    left.whenPressed(() -> SmartDashboard.putNumber("hood_preset", 1));
  }

  public void setRumble(double rumble) {
    controller.setRumble(RumbleType.kLeftRumble, rumble);
    controller.setRumble(RumbleType.kRightRumble, rumble);
  }
  
  public Command getAutonomousCommand() {
    return autonomousSelector.getSelected();
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
