// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionModule;

public class Shoot extends CommandBase {
  private Shooter shooter;
  private VisionModule vision;
  private boolean useCV;
  
  public Shoot(Shooter shooter, VisionModule vision, boolean useCV) {
    this.shooter = shooter;
    this.vision = vision;
    this.useCV = useCV;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.setShooterRPM(5500);

    if(useCV && vision.hasTarget()) {
      shooter.setHoodAngle(MathUtil.clamp(vision.getTargetDistance() * 3.28 + 8.0, 0, 36));
    } else {
      shooter.setHoodAngle(MathUtil.clamp(SmartDashboard.getNumber("goal_distance", 4.5) * 3.28 + 8.0, 0, 36));
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterSpeed(0);
    shooter.setHoodSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
