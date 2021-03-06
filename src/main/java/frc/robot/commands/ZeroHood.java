/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ZeroHood extends CommandBase {
  private Shooter shooter;
  
  public ZeroHood(Shooter shooter) {
    this.shooter = shooter;
    
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    if(!shooter.isHoodRetracted()) {
      shooter.setHoodSpeed(0.1);
    } else {
      shooter.setHoodSpeed(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setHoodSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
