// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

public final class Constants {
	public static final class ShooterConstants {
		public static final int LEADER_FLYWHEEL_TALON = 6;
		public static final int FOLLOWER_FLYWHEEL_TALON = 7;
		
		public static final int HOOD_SPARK = 11;

		public static final int ACCELERATOR_SPARK = 8;

		public static final double encoderRotationsPerHoodDegree = -1.093792;
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

		public static final double ksVolts = 0.138;
		public static final double kvVoltSecondsPerMeter = 0.798;
		public static final double kaVoltSecondsSquaredPerMeter = 0.0389;
	
		public static final double kPDriveVel = 0.085;
	
		public static final double kTrackwidthMeters = Units.feetToMeters(2.34);
		public static final double wheelDiameterMeters = Units.inchesToMeters(6);

		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
	}

	public static final class VisionConstants {
		public static final String CAMERA_NAME = "mmal_service_16.1";

		public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(12);
		public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(82);
		public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(20);
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

		public static final int LEFT_BUMPER = 5;
		public static final int RIGHT_BUMPER = 6;

		public static final int LEFT_JOYSTICK = 1;
		public static final int RIGHT_JOYSTICK = 4;

		public static final int A_BUTTON = 1;
		public static final int B_BUTTON = 2;
		public static final int X_BUTTON = 3;

		public static final int UP = 0;
		public static final int DOWN = 180;
	
		public static final double JOYSTICK_DEADBAND = 0.05;
	}
}
