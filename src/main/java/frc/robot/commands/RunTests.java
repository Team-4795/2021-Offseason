// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RunTests extends CommandBase {
  private Drivebase drivebase;
  private Intake intake;
  private Indexer indexer;
  private Shooter shooter;
  
  public RunTests(Drivebase drivebase, Intake intake, Indexer indexer, Shooter shooter) {
    this.drivebase = drivebase;
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;

    addRequirements(drivebase);
    addRequirements(intake);
    addRequirements(indexer);
    addRequirements(shooter);
  }
  
  @Override
  public void initialize() {
    drivebase.curvatureDrive(0.25, 0, false);

    intake.setIntakeSpeed(0.5);

    indexer.setIndexerSpeed(0.25, 0.25);

    shooter.setAcceleratorRPM(1000);
    shooter.setShooterRPM(1000);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    drivebase.curvatureDrive(0, 0, false);

    intake.setIntakeSpeed(0);

    indexer.setIndexerSpeed(0, 0);

    shooter.setShooter(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
