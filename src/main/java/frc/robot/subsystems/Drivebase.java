// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Drivebase extends SubsystemBase {
  private CANSparkMax leftLeader = new CANSparkMax(DrivebaseConstants.LEFT_LEADER, MotorType.kBrushless);
  private CANSparkMax leftFollower = new CANSparkMax(DrivebaseConstants.LEFT_FOLLOWER, MotorType.kBrushless);

  private CANSparkMax rightLeader = new CANSparkMax(DrivebaseConstants.RIGHT_LEADER, MotorType.kBrushless);
  private CANSparkMax rightFollower = new CANSparkMax(DrivebaseConstants.RIGHT_FOLLOWER, MotorType.kBrushless);

  private DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader);

  private CANEncoder leftEncoder, rightEncoder;

  private double movementSpeed = 0;
  private int direction = 1;

  private double startLeft, startRight;

  private static final double countsPerRevolution = 2048;
  private static final double wheelDiameterMeter = 0.1524;

  private AHRS gyro;

  private final DifferentialDriveOdometry odometry;

  public Drivebase() {
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);

    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    leftEncoder.setPositionConversionFactor((Math.PI * wheelDiameterMeter) / countsPerRevolution);
    leftEncoder.setVelocityConversionFactor((Math.PI * wheelDiameterMeter) / countsPerRevolution);
    rightEncoder.setPositionConversionFactor((Math.PI * wheelDiameterMeter) / countsPerRevolution);
    rightEncoder.setVelocityConversionFactor((Math.PI * wheelDiameterMeter) / countsPerRevolution);

    startLeft = leftEncoder.getPosition();
    startRight = rightEncoder.getPosition();

    gyro = new AHRS(SPI.Port.kMXP);

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    diffDrive.setDeadband(0.0);
  }

  public void curvatureDrive(double speed, double rotation, boolean quickTurn) {
    movementSpeed = Math.max(Math.abs(speed), Math.abs(rotation));

    diffDrive.curvatureDrive(speed * direction, rotation, quickTurn);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(-rightVolts);
    diffDrive.feed();
  }

  public void reverse() {
    if(Math.abs(movementSpeed) < 0.3) direction *= -1;
  }

  public void resetEncoders() {
    startLeft = leftEncoder.getPosition();
    startRight = rightEncoder.getPosition();
  }

  public double getLeftEncoder() {
    return leftEncoder.getPosition() - startLeft;
  }

  public double getRightEncoder() {
    return rightEncoder.getPosition() - startRight;
  }

  public double getSpeed() {
    return movementSpeed;
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    SmartDashboard.putNumber("Left drivebase encoder", getLeftEncoder());
    SmartDashboard.putNumber("Right drivebase encoder", getRightEncoder());
  }
}
