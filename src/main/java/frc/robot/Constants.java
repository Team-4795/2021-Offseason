// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {
	public static final class ShooterConstants {
		public static final int LEADER_FLYWHEEL_TALON = 6;
		public static final int FOLLOWER_FLYWHEEL_TALON = 7;
		
		public static final int HOOD_SPARK = 11;

		public static final int ACCELERATOR_SPARK = 8;
	}

	public static final class IntakeConstants {
		public static final int INTAKE_SPARK = 0;
	
		public static final int FORWARD_SOLENOID = 6;
		public static final int REVERSE_SOLENOID = 7;
	}
	
	public static final class IndexerConstants {
		public static final int INDEXER_VICTOR = 9;
		public static final int SELECTOR_VICTOR = 10;
	}

	public static final class DrivebaseConstants {
		public static final int LEFT_LEADER = 4;
		public static final int LEFT_FOLLOWER = 5;
		public static final int RIGHT_LEADER = 2;
		public static final int RIGHT_FOLLOWER = 3;

		public static final double ksVolts = 0.929;
		public static final double kvVoltSecondsPerMeter = 6.33;
		public static final double kaVoltSecondsSquaredPerMeter = 0.0389;
	
		public static final double kPDriveVel = 0.085;
	
		public static final double kTrackwidthMeters = 0.142072613;
		public static final double wheelDiameterMeter = 0.1524;
		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
	}

	public static final class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 0.5;
		public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
		
		public static final double kRamseteB = 2;
		public static final double kRamseteZeta = 0.7;
	}

	public static final class ControllerConstants {
		public static final int LEFT_TRIGGER = 2;
		public static final int RIGHT_TRIGGER = 3;
		public static final int LEFT_BUMPER = 4;
		public static final int RIGHT_BUMPER = 5;
		public static final int LEFT_JOYSTICK = 1;
		public static final int RIGHT_JOYSTICK = 4;
		public static final int A_BUTTON = 1;
		public static final int B_BUTTON = 0;
	
		public static final double JOYSTICK_DEADBAND = 0.05;
	}
}
