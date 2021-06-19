// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class Index extends CommandBase {
  private Indexer indexer;

  public Index(Indexer indexer) {
    this.indexer = indexer;
    
    addRequirements(indexer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    indexer.setIndexerSpeed(0.3, 0.75);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.setIndexerSpeed(0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
