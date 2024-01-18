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
    public static final double kWheelBase = Units.inchesToMeters(24);
    // Distance between front and back wheels


public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
     
    new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //front left
      new Translation2d(kWheelBase / 2, kTrackWidth / 2), //front right
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //back left
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); //back right

//start front front left to front right to back right and all drives then all steers then all absolutes

    public static final int kFrontLeftDriveMotorPort = 3;
    public static final int kFrontRightDriveMotorPort = 6;
    public static final int kBackRightDriveMotorPort = 9;
    public static final int kBackLeftDriveMotorPort = 11;

    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kFrontRightTurningMotorPort = 7;
    public static final int kBackLeftTurningMotorPort = 12;
    public static final int kBackRightTurningMotorPort = 10;
    
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 4;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 13;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 5;
    public static final int kBackRightDriveAbsoluteEncoderPort = 8;
    
    public static final double kBRDegrees = 96.591;//9.405
    public static final double kBLDegrees = 26.367;//294.875
    public static final double kFLDegrees = 158.818;//68.82
    public static final double kFRDegrees = -26.718;//242.93

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = true;

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
    public static final double kDriveMotorGearRatio = 7 / 1;
    public static final double kTurningMotorGearRatio = 12.8 / 1;
    public static final double kDriveEncoderRot2Meter = 1/23.58;
    
    
    //get this a little closer. maybe 28.23
    public static final double kTurningConversionFactor2Deg =  28.25;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2DegPerSec = kTurningConversionFactor2Deg / 60;

    public static final double kPTurning = 0.01; //test a higher value 0.025 is to high
    public static final double kITurning = 0.00015;
    public static final double kDTurning = 0.05;

    public static final double moduleRadius = 0.4318; //meters -- measured from center of robot to furthest module.
}

//Max speed
    public static final class OIConstants {
      public static final int kOPControllerPort = 1;
      public static final double kDeadband = 0.09;
  }
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    public static final double kMaxAngularSpeedRadiansPerSecond =  DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    public static final double kMaxAccelerationMetersPerSecondSquared = DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond;
    public static final double kMaxAngularAccelerationUnitsPerSecond = DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond;

    public static final double kPTranslation = 0.5;
    public static final double kITranslation = 0;
    public static final double kDTranslation = 0;

    public static final double kPTheta = 1;
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
//TODO Change these cause they are gonna be wrong
  public static final int intLeftEncoder = 13;
  public static final int intRightEncoder = 14;
  public static final int handoffEncoder = 15;

  public static final double ramp_rate = 0.5;

  public static final boolean leftInverted = false;
  public static final boolean rightInverted = false;
  public static final boolean handoffInverted = false;

  public static final double kP_Intake = 0.05;
  public static final double kI_Intake = 0;
  public static final double kD_Intake = 0;

  public static final double kP_Handoff = 0.05;
  public static final double kI_Handoff = 0;
  public static final double kD_Handoff = 0;

  public static final int liftNum = 1;
  public static final int dropNum = 2;

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
  public static final int fwLeftMotorNum = 17;
  public static final int fwRightMotorNum = 18;

  public static final double ramp_rate = 0.1;

  public static final boolean fwLeftInverted = false;
  public static final boolean fwRightInverted = false;

  public static final double kP_Shooter = 0.5;
  public static final double kI_Shooter = 0;
  public static final double kD_Shooter = 0;

}

public static final class LateratorConstants{
  public static final int leftMotor = 19;
  public static final int rightMotor = 20;

  public static final double ramp_rate = 1;

  public static final boolean leftInverted = false;
  public static final boolean rightInverted = false;

  public static final double kP_Laterator = 0.5;
  public static final double kI_Laterator = 0;
  public static final double kD_Laterator = 0;
}

}
