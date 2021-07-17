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
  public void execute() {
    if(useCV && vision.hasTarget()) {
      shooter.setShooterRPM(3700);
      shooter.setHoodAngle(MathUtil.clamp(vision.getTargetDistance() * 6.0 / 13.0 + 460.0 / 13.0, 0, 45));
    } else {
      double hoodPreset = SmartDashboard.getNumber("hood_preset", 1);

      if(hoodPreset == 0) {
        shooter.setShooterRPM(3900);
        shooter.setHoodAngle(3);
      } else if(hoodPreset == 1) {
        shooter.setShooterRPM(3900);
        shooter.setHoodAngle(42);
      } else if(hoodPreset == 2) {
        shooter.setShooterRPM(3900);
        shooter.setHoodAngle(45);
      }
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
