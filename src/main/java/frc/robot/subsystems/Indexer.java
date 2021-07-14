// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  private VictorSPX indexerMotor = new VictorSPX(IndexerConstants.INDEXER_VICTOR);
  private VictorSPX selectorMotor = new VictorSPX(IndexerConstants.SELECTOR_VICTOR);

  public Indexer() {
    indexerMotor.configOpenloopRamp(1.25, 0);
    selectorMotor.configOpenloopRamp(1.25, 0);
  }

  public void setIndexerSpeed(double indexerSpeed, double selectorSpeed) {
    indexerMotor.set(VictorSPXControlMode.PercentOutput, indexerSpeed);
    selectorMotor.set(VictorSPXControlMode.PercentOutput, selectorSpeed);
  }
}
