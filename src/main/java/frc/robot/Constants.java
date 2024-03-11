// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    public static final double kTrackWidth = Units.inchesToMeters(23);
      // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(23);
      // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //front left
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), //front right
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //back left
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); //back right
    //start front front left to front right to back right and all drives then all steers then all absolutes
    public static final int kFrontRightDriveMotorPort = 1;
    public static final int kFrontRightTurningMotorPort = 2;
    public static final int kBackRightDriveMotorPort = 3;
    public static final int kBackRightTurningMotorPort = 4;
    public static final int kBackLeftTurningMotorPort = 5;
    public static final int kBackLeftDriveMotorPort = 6;
    public static final int kFrontLeftDriveMotorPort = 7;
    public static final int kFrontLeftTurningMotorPort = 8;

      
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
    public static final int kBackRightDriveAbsoluteEncoderPort = 11;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 12;

    public static final double kFLDegrees = 277.21;
    public static final double kFRDegrees = 217.529;
    public static final double kBRDegrees = 155.4785;
    public static final double kBLDegrees = 45.791;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.572;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 1.35;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3.5;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2.5;
  }
  
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 6.75 / 1;
    public static final double kTurningMotorGearRatio = 12.8 / 1;
    public static final double kDriveEncoderRot2Meter = 1/23.58;
    
    public static final double kTurningConversionFactor2Deg =  28.25;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2DegPerSec = kTurningConversionFactor2Deg / 60;

    public static final double kPTurning = 0.0075;
    public static final double kITurning = 0;
    public static final double kDTurning = 0.75;

    public static final double moduleRadius = 0.4318; //meters -- measured from center of robot to furthest module.
  }


  public static final class OIConstants {
    public static final int kOPControllerPort = 1;
    public static final double kDeadband = 0.09;
  }


  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    public static final double kMaxAngularSpeedRadiansPerSecond =  DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    public static final double kMaxAccelerationMetersPerSecondSquared = DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond;
    public static final double kMaxAngularAccelerationUnitsPerSecond = DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond;

    public static final double kPTranslation = 0.08;
    public static final double kITranslation = 0.005;
    public static final double kDTranslation = 0;

    public static final double kPTheta = 0.05;
    public static final double kITheta = 0;
    public static final double kDTheta = 0;


    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
            new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond,
                    kMaxAngularAccelerationUnitsPerSecond);
    public static final TrapezoidProfile.Constraints kLinearConstraints = 
            new TrapezoidProfile.Constraints(
                kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared
            );
  }


  public static final class intakeConstants {
    public static final int intLeftEncoder = 13;
    public static final int intRightEncoder = 14;
    public static final int indexingSensor = 1;

    public static final double ramp_rate = 0.5;

    public static final boolean leftInverted = true;
    public static final boolean rightInverted = true;

    public static final double kP_Intake = 0.05;
    public static final double kI_Intake = 0;
    public static final double kD_Intake = 0;

    public static final int liftNum = 0;
    public static final int dropNum = 1;

  }

  public static final class limelightConstants{
    public static final double thetakP = 3.0;
    public static final double thetakI = 0;
    public static final double thetakD = 0;

    public static final double linearkP = 0.4;
    public static final double linearkI = 0;
    public static final double linearkD = 0;

  }


  public static final class ShooterConstants{
    //CAN Bus Numbers
    public static final int topMotorNum = 16;
    public static final int bottomMotorNum = 17;
    public static final int lPitchEncoder = 18;
    public static final int rPitchEncoder = 19;

    public static final double ramp_rate = 0.1;

  
    //PID Values
    public static final double kS_TopShooter = 0.4;
    public static final double kV_TopShooter = 0.12;
    public static final double kA_TopShooter = 0.1;
    public static final double kP_TopShooter = 0.6;
    public static final double kI_TopShooter = 0;
    public static final double kD_TopShooter = 0.01;
    
    public static final double kS_BottomShooter = 0.4;
    public static final double kV_BottomShooter = 0.12;
    public static final double kA_BottomShooter = 0.1;
    public static final double kP_BottomShooter = 0.6;
    public static final double kI_BottomShooter = 0;
    public static final double kD_BottomShooter = 0.01;

    public static final double kP_pitch = 0.01;
    public static final double kI_pitch = 0;
    public static final double kD_pitch = 0;

    //Encoder values
    public static final double toDegrees = 360;
    public static final double pitchOffset = 0;
    public static final boolean lPitchReversed = false;
    public static final boolean rPitchReversed = false;
    public static final boolean topMotorReversed = false;
    public static final boolean bottomMotorReversed = false;


    //Known Angles
    public static final double sourceAngle = 0;
    public static final double closeSpeakerAngle = 0;
    public static final double podiumSpeakerAngle = 0;
    public static final double ampAngle = 0;
  }


}
