// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionModule extends SubsystemBase {
  private PhotonCamera camera = new PhotonCamera(VisionConstants.CAMERA_NAME);

  private boolean hasTarget = false;
  private double targetDistance = 0;
  private double targetAngle = 0;

  public boolean hasTarget() {
    return hasTarget;
  }

  public double getTargetDistance() {
    return targetDistance;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  @Override
  public void periodic() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      hasTarget = true;
      targetDistance =
              Units.metersToFeet(PhotonUtils.calculateDistanceToTargetMeters(
                      VisionConstants.CAMERA_HEIGHT_METERS,
                      VisionConstants.TARGET_HEIGHT_METERS,
                      VisionConstants.CAMERA_PITCH_RADIANS,
                      Units.degreesToRadians(result.getBestTarget().getPitch())));
      targetAngle = result.getBestTarget().getYaw();
    } else {
      hasTarget = false;
      targetDistance = -1;
      targetAngle = -1;
    }
  }
}
