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

  public TurnToGoal(Drivebase drivebase, VisionModule vision) {
    this.drivebase = drivebase;
    this.vision = vision;

    addRequirements(drivebase);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(!vision.hasTarget()) return;

    double angle = vision.getTargetAngle();
    double turnSpeed = angle / 20.0;
    
    turnSpeed = MathUtil.clamp(Math.max(Math.abs(turnSpeed), 0.1) * Math.signum(turnSpeed), -0.8, 0.8);

    if(angle > 0.5) drivebase.curvatureDrive(0, turnSpeed, true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}