// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class Index extends CommandBase {
  private Indexer indexer;
  private double indexerSpeed;
  private double selectorSpeed;
  private double delay;
  private long startTime;

  public Index(Indexer indexer, double indexerSpeed, double selectorSpeed, double delay) {
    this.indexer = indexer;
    this.indexerSpeed = indexerSpeed;
    this.selectorSpeed = selectorSpeed;
    this.delay = delay;
    
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(System.currentTimeMillis() - startTime > delay) indexer.setIndexerSpeed(indexerSpeed, selectorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setIndexerSpeed(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}