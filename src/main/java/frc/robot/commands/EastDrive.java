// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import java.util.function.Supplier;

public class EastDrive extends CommandBase {
  private final Drivebase m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;
  private final Supplier<Double> m_throttle;
  private long lastAcceleration;
  private long lastUpdate;
  private double speed = 0;
  private double minSpeed = 0.05;

  public EastDrive(
      Drivebase drivetrain,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSupplier,
      Supplier<Double> throttle) {
    m_drivetrain = drivetrain;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSupplier;
    m_throttle = throttle;
    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    lastAcceleration = 0;
    lastUpdate = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    double acceleration = m_xaxisSpeedSupplier.get();
    double rotation = m_zaxisRotateSupplier.get();
    double throttle = m_throttle.get();

    acceleration = Math.min(0.5 * acceleration + 0.5 * Math.pow(acceleration, 3), 0.75);
    throttle = (1.0 - throttle * 0.65);
    
    if(Math.abs(acceleration) > 0) {
      lastAcceleration = System.currentTimeMillis();

      if(Math.signum(speed) != Math.signum(acceleration)) {
        speed = Math.copySign(minSpeed, acceleration);
      } else {
        double adjustment = (System.currentTimeMillis() - lastUpdate) / 500.0;
        
        speed = Math.min(Math.abs(speed) + adjustment, Math.abs(acceleration) * (1.0 - minSpeed) + minSpeed);
        speed = Math.copySign(Math.max(speed * throttle, minSpeed), acceleration);
      }
    } else {
      speed = 0;
    }
    
    if(speed == 0) {
      double transitionRamp = MathUtil.clamp((System.currentTimeMillis() - lastAcceleration) / 500.0, 0.5, 1.0);

      rotation *= Math.max(throttle, 0.4) * transitionRamp * 0.4;

      m_drivetrain.curvatureDrive(speed, rotation, true);
    } else {
      m_drivetrain.curvatureDrive(speed, rotation, false);
    }

    lastUpdate = System.currentTimeMillis();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
