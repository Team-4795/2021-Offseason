// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.VisionModule;

public class TurnToGoal extends CommandBase {
  private Drivebase drivebase;
  private VisionModule vision;
  
  private long lastUpdate;

  public TurnToGoal(Drivebase drivebase, VisionModule vision) {
    this.drivebase = drivebase;
    this.vision = vision;

    lastUpdate = System.currentTimeMillis();

    addRequirements(drivebase);
  }

  @Override
  public void execute() {
    if(!vision.hasTarget()) return;

    double angle = vision.getTargetAngle();
    double turnSpeed = -angle / 400.0;
    
    turnSpeed = MathUtil.clamp(Math.copySign(Math.max(Math.abs(turnSpeed), 0.035), turnSpeed), -0.15, 0.15);

    if(Math.abs(angle + 1) > 1) {
      drivebase.curvatureDrive(0, turnSpeed, true);

      lastUpdate = System.currentTimeMillis();
    } else {
      drivebase.curvatureDrive(0, 0, false);
    }
  }

  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - lastUpdate > 250;
  }
}