package frc.robot.subsystems;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.OIConstants;

public class SwerveSubsystem extends SubsystemBase{
    public final CommandXboxController opController = new CommandXboxController(OIConstants.kOPControllerPort);
    public boolean isRedAlliance;
    Optional<Alliance> alliance;
    public boolean fieldOriented;
    public Robot robot;

    public static SwerveModule frontLeftModule = new SwerveModule(Constants.DriveConstants.kFrontLeftTurningMotorPort, Constants.DriveConstants.kFrontLeftDriveMotorPort, Constants.DriveConstants.kFrontLeftDriveEncoderReversed, Constants.DriveConstants.kFrontLeftTurningEncoderReversed, Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, Constants.DriveConstants.kBLDegrees, Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
    public static SwerveModule frontRightModule = new SwerveModule(Constants.DriveConstants.kFrontRightTurningMotorPort, Constants.DriveConstants.kFrontRightDriveMotorPort, Constants.DriveConstants.kFrontRightDriveEncoderReversed, Constants.DriveConstants.kFrontRightTurningEncoderReversed, Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderPort, Constants.DriveConstants.kBRDegrees, Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
    public static SwerveModule backLeftModule = new SwerveModule(Constants.DriveConstants.kBackLeftTurningMotorPort, Constants.DriveConstants.kBackLeftDriveMotorPort, Constants.DriveConstants.kBackLeftDriveEncoderReversed, Constants.DriveConstants.kBackLeftTurningEncoderReversed, Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderPort, Constants.DriveConstants.kFLDegrees, Constants.DriveConstants.kBackLeftTurningEncoderReversed);
    public static SwerveModule backRightModule = new SwerveModule(Constants.DriveConstants.kBackRightTurningMotorPort, Constants.DriveConstants.kBackRightDriveMotorPort, Constants.DriveConstants.kBackRightDriveEncoderReversed, Constants.DriveConstants.kBackRightTurningEncoderReversed, Constants.DriveConstants.kBackRightDriveAbsoluteEncoderPort, Constants.DriveConstants.kFRDegrees, Constants.DriveConstants.kBackRightTurningEncoderReversed);
   
    //reset all method. Used after face foward to then reference all RE values as 0 being the front.
public void ResetAllEncoders() {
    frontLeftModule.resetDrive();
    frontRightModule.resetDrive();
    backLeftModule.resetDrive();
    backRightModule.resetDrive();
}

//gyro int and heading code
    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    public final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics,
    geRotation2d(), new SwerveModulePosition[] {
 frontLeftModule.getPosition(),
 frontRightModule.getPosition(),
 backLeftModule.getPosition(),
backRightModule.getPosition()
    });
    
    public SwerveSubsystem(Robot robot) {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {}}).
            start();

        this.robot = robot;
        alliance = robot.getAlliance();
    }

    //used to zero the gyro and used to refrence where the far end of the field is during comp.
    public void zeroHeading() {
        gyro.reset();
        gyro.setAngleAdjustment(180);
    }

    public void zeroHeadingButton() {
        gyro.reset();
        gyro.setAngleAdjustment(0);
    }

    public Command zeroHeadingCommand() {
        return runOnce(() -> {zeroHeadingButton();});
    }

    //used for debugging and field centric
    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    //used for Field Centric
    public Rotation2d geRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(geRotation2d(), new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()}, pose);
    }

    public boolean allianceCheck(){
        if (alliance.isPresent() && (alliance.get() == Alliance.Red)) {isRedAlliance = true;}else{isRedAlliance = false;}
        return isRedAlliance;
    }

    @Override
    public void periodic() {

        

    odometer.update(geRotation2d(),  new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
           backRightModule.getPosition()
        });

        




//multiple debugging values are listed here. Names are self explanitory

        //Odometer and other gyro values
       SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
       SmartDashboard.putNumber("Roll value", getRoll());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("R2d", geRotation2d().getDegrees());

        //AE Degrees Reading
        SmartDashboard.putNumber("Back Left AE Value", backLeftModule.getAbsoluteEncoderDeg(Constants.DriveConstants.kBLDegrees));
        SmartDashboard.putNumber("Back Right AE Value", backRightModule.getAbsoluteEncoderDeg(Constants.DriveConstants.kBRDegrees));
        SmartDashboard.putNumber("Front Left AE Value", frontLeftModule.getAbsoluteEncoderDeg(Constants.DriveConstants.kFLDegrees));
        SmartDashboard.putNumber("Front Right AE Value", frontRightModule.getAbsoluteEncoderDeg(Constants.DriveConstants.kFRDegrees));
    //    //RE Degrees Reading
        // SmartDashboard.putNumber("Back left RE Value", backLeftModule.getSteerPosition());
        // SmartDashboard.putNumber("Back Right RE Value", backRightModule.getSteerPosition());
        // SmartDashboard.putNumber("Front left RE Value", frontLeftModule.getSteerPosition());
        // SmartDashboard.putNumber("Front Right RE Value", frontRightModule.getSteerPosition());

    //    SmartDashboard.putNumber("Front Left Drive Position", frontLeftModule.getDrivePosition());
    //    SmartDashboard.putNumber("Front Right Drive Position", frontRightModule.getDrivePosition());
    //    SmartDashboard.putNumber("Back Left Drive Position", backLeftModule.getDrivePosition());
    //    SmartDashboard.putNumber("Back Right Drive Position", backRightModule.getDrivePosition());
    
//end of periodic phase
}

//stops all modules. Called when the command isn't being ran. So when an input isn't recieved
    public void stopModules() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }

    public void setModuleStates(ChassisSpeeds speeds){
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeftModule.setDesiredState(moduleStates[1]);
       frontRightModule.setDesiredState(moduleStates[0]);
       backLeftModule.setDesiredState(moduleStates[3]);
       backRightModule.setDesiredState(moduleStates[2]);
    }

    public Command faceForwardCommand() {
        return runOnce(() -> {faceAllFoward();});
    }

   //face forward method. Called once the bot is enabled
public void faceAllFoward() {
    backRightModule.wheelFaceForward(Constants.DriveConstants.kBRDegrees);
    frontLeftModule.wheelFaceForward(Constants.DriveConstants.kFLDegrees);
    frontRightModule.wheelFaceForward(Constants.DriveConstants.kFRDegrees);
   backLeftModule.wheelFaceForward(Constants.DriveConstants.kBLDegrees);
    System.out.println("exacuted faceAll");
}

public Command fieldOrientedToggle(){
    return runOnce(() -> {fieldOriented = !fieldOriented;});
}

public Command resetWheels(){
    return runOnce(() -> {
        faceForwardCommand();
        try{
            Thread.sleep(50);
            frontLeftModule.steerMotorEncoder.setPosition(0);
            frontRightModule.steerMotorEncoder.setPosition(0);
            backLeftModule.steerMotorEncoder.setPosition(0);
            backRightModule.steerMotorEncoder.setPosition(0);
        }catch (Exception i) {}});
}

//Commands for Autos
public Command eventHit(){
    return runOnce(() -> {SmartDashboard.putString("Event", "Passed Marker");});
  }
}
