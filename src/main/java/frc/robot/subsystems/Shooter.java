// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private CANSparkMax acceleratorWheel = new CANSparkMax(ShooterConstants.ACCELERATOR_SPARK, MotorType.kBrushless);
    
  private TalonFX leaderFlywheel = new TalonFX(ShooterConstants.LEADER_FLYWHEEL_TALON);
  private TalonFX followerFlywheel = new TalonFX(ShooterConstants.FOLLOWER_FLYWHEEL_TALON);

  private CANSparkMax hood = new CANSparkMax(ShooterConstants.HOOD_SPARK, MotorType.kBrushless);

  private CANDigitalInput hoodLimit;

  private CANPIDController acceleratorController; 
  private CANPIDController hoodController;

  private CANEncoder hoodEncoder;

  public Shooter() {
    acceleratorWheel.restoreFactoryDefaults();
    hood.restoreFactoryDefaults();    
    leaderFlywheel.configFactoryDefault();

    acceleratorController = acceleratorWheel.getPIDController();
    hoodController = hood.getPIDController();

    hoodEncoder = hood.getEncoder();
    hoodEncoder.setPositionConversionFactor(-1);

    hoodLimit = hood.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

    hood.setInverted(true);
    hood.setSoftLimit(SoftLimitDirection.kReverse, -51.5f); 
    hood.enableSoftLimit(SoftLimitDirection.kReverse, true);

    leaderFlywheel.setInverted(true);
    followerFlywheel.follow(leaderFlywheel);

    leaderFlywheel.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    followerFlywheel.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    
    acceleratorController.setP(0.01);
    acceleratorController.setFF(0.1);
    acceleratorController.setOutputRange(-1, 1);

    hoodController.setP(0.2);
    hoodController.setOutputRange(-0.5, 0.5);

    leaderFlywheel.config_kF(0, 0.0485, 0);
    leaderFlywheel.config_kP(0, 0.016, 0);
  }

  public void resetHoodEncoder() {
    hoodEncoder.setPosition(0.0);
  }

  public void setHoodSpeed(double speed) {
    hood.set(speed);
  }

  public void setHoodAngle(double degrees) {
    hoodController.setReference(degrees * ShooterConstants.encoderRotationsPerHoodDegree, ControlType.kPosition);
  }

  public boolean isHoodRetracted() {
    return hoodLimit.get();
  }

  public void setShooterSpeed(double speed) {
    leaderFlywheel.set(ControlMode.PercentOutput, speed);
    acceleratorWheel.set(speed);
  }

  public void setShooterRPM(double speed) {
    // 2048 ticks per revolution 
    // ticks per .10 second 
    // 1 / 2048 * 60
    double speed_FalconUnits = speed / (600.0) * 2048.0;
    leaderFlywheel.set(TalonFXControlMode.Velocity, speed_FalconUnits);
    acceleratorController.setReference(speed, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    if (hoodLimit.get()) {
      hoodEncoder.setPosition(0);
    }
    
    SmartDashboard.putBoolean("limit switch", hoodLimit.get());
    SmartDashboard.putNumber("shooter speed", (leaderFlywheel.getSelectedSensorVelocity()) / 2048.0 * 600);
    SmartDashboard.putNumber("hood angle", hoodEncoder.getPosition() / ShooterConstants.encoderRotationsPerHoodDegree);
  }
}
