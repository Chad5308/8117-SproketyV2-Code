package frc.robot.subsystems.Drivebase;

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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SwerveSubsystem extends SubsystemBase{
    public final CommandXboxController opController = new CommandXboxController(OIConstants.kOPControllerPort);
    public boolean fieldOriented = false;
    public Robot robot;
    public ShooterSubsystem shoot_sub;
    public IntakeSubsystem arm_sub;
    public boolean isRedAlliance;
    Optional<Alliance> alliance;

    public static SwerveModule frontRightModule = new SwerveModule(Constants.DriveConstants.kFrontRightTurningMotorPort, Constants.DriveConstants.kFrontRightDriveMotorPort, Constants.DriveConstants.kFrontRightDriveEncoderReversed, Constants.DriveConstants.kFrontRightTurningEncoderReversed, Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderPort, Constants.DriveConstants.kBRDegrees, Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
    public static SwerveModule frontLeftModule = new SwerveModule(Constants.DriveConstants.kFrontLeftTurningMotorPort, Constants.DriveConstants.kFrontLeftDriveMotorPort, Constants.DriveConstants.kFrontLeftDriveEncoderReversed, Constants.DriveConstants.kFrontLeftTurningEncoderReversed, Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, Constants.DriveConstants.kBLDegrees, Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
    public static SwerveModule backRightModule = new SwerveModule(Constants.DriveConstants.kBackRightTurningMotorPort, Constants.DriveConstants.kBackRightDriveMotorPort, Constants.DriveConstants.kBackRightDriveEncoderReversed, Constants.DriveConstants.kBackRightTurningEncoderReversed, Constants.DriveConstants.kBackRightDriveAbsoluteEncoderPort, Constants.DriveConstants.kFRDegrees, Constants.DriveConstants.kBackRightTurningEncoderReversed);
    public static SwerveModule backLeftModule = new SwerveModule(Constants.DriveConstants.kBackLeftTurningMotorPort, Constants.DriveConstants.kBackLeftDriveMotorPort, Constants.DriveConstants.kBackLeftDriveEncoderReversed, Constants.DriveConstants.kBackLeftTurningEncoderReversed, Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderPort, Constants.DriveConstants.kFLDegrees, Constants.DriveConstants.kBackLeftTurningEncoderReversed);
   
    
    public SwerveSubsystem(Robot robot, ShooterSubsystem shoot_sub, IntakeSubsystem arm_sub) {
        new Thread(() -> {
            try {
                Thread.sleep(500);
                zeroHeading();
            } catch (Exception e) {}}).start();
            
            this.robot = robot;
            this.arm_sub = arm_sub;
            this.shoot_sub = shoot_sub;
            alliance = robot.getAlliance();
            }


    public boolean allianceCheck(){
        if (alliance.isPresent() && (alliance.get() == Alliance.Red)) {isRedAlliance = true;}else{isRedAlliance = false;}
        return isRedAlliance;
    }
            
    
        
    //gyro int and heading code
    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    public void zeroHeading() {
        gyro.reset();
        gyro.setAngleAdjustment(0);
    }
    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }
    public Rotation2d geRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    //Odometer code
    public final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics,
    geRotation2d(), new SwerveModulePosition[] {
    frontLeftModule.getPosition(),
    frontRightModule.getPosition(),
    backLeftModule.getPosition(),
    backRightModule.getPosition()
    });

    public SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[]{frontRightModule.getPosition(), frontLeftModule.getPosition(), backRightModule.getPosition(), backLeftModule.getPosition()};

    public void resetPositions(SwerveModule FL, SwerveModule FR, SwerveModule BL, SwerveModule BR){
        FL.resetDriveEncoder();
        FR.resetDriveEncoder();
        BL.resetDriveEncoder();
        BR.resetDriveEncoder();
    }
    public void resetOdometry(Pose2d pose) {
        resetPositions(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
        odometer.resetPosition(geRotation2d(), swerveModulePositions, new Pose2d());
        odometer.resetPosition(geRotation2d(), swerveModulePositions, pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        SwerveModuleState[] states = new SwerveModuleState[]{frontRightModule.gState(), frontLeftModule.gState(), backRightModule.gState(), backLeftModule.gState()};
        return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(states);
    }
    public Pose2d getPose() {
        return odometer.getPoseMeters();
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
        frontRightModule.setDesiredState(moduleStates[0]);
        frontLeftModule.setDesiredState(moduleStates[1]);
        backRightModule.setDesiredState(moduleStates[2]);
        backLeftModule.setDesiredState(moduleStates[3]);
    }
    public void driveRobotRelative(ChassisSpeeds speeds){
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // frontLeftModule.setDesiredState(moduleStates[0]);//fr
        // backLeftModule.setDesiredState(moduleStates[1]);//fl
        // frontRightModule.setDesiredState(moduleStates[2]);//br
        // backRightModule.setDesiredState(moduleStates[3]);//bl
        frontRightModule.setDesiredState(moduleStates[0]);
        frontLeftModule.setDesiredState(moduleStates[1]);
        backRightModule.setDesiredState(moduleStates[2]);
        backLeftModule.setDesiredState(moduleStates[3]);
    }
    
    //face forward method. Called once the bot is enabled
    public void faceAllFoward() {
        backRightModule.wheelFaceForward(Constants.DriveConstants.kBRDegrees);
        frontLeftModule.wheelFaceForward(Constants.DriveConstants.kFLDegrees);
        frontRightModule.wheelFaceForward(Constants.DriveConstants.kFRDegrees);
        backLeftModule.wheelFaceForward(Constants.DriveConstants.kBLDegrees);
        System.out.println("exacuted faceAll");
    }
    
    public Command resetWheels(){
        return runOnce(() -> {
                frontLeftModule.wheelFaceForward(Constants.DriveConstants.kFLDegrees);
                frontRightModule.wheelFaceForward(Constants.DriveConstants.kFRDegrees);
                backLeftModule.wheelFaceForward(Constants.DriveConstants.kBLDegrees);
                backRightModule.wheelFaceForward(Constants.DriveConstants.kBRDegrees);
    });}
        
    public Command fieldOrientedToggle(){
        return runOnce(() -> {fieldOriented = !fieldOriented;});
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
               SmartDashboard.putNumber("Robot Heading", getHeading());
        
                //AE Degrees Reading
            //     SmartDashboard.putNumber("Back Left AE Value", backLeftModule.getAbsoluteEncoderDeg(Constants.DriveConstants.kBLDegrees));
            //     SmartDashboard.putNumber("Back Right AE Value", backRightModule.getAbsoluteEncoderDeg(Constants.DriveConstants.kBRDegrees));
            //     SmartDashboard.putNumber("Front Left AE Value", frontLeftModule.getAbsoluteEncoderDeg(Constants.DriveConstants.kFLDegrees));
            //     SmartDashboard.putNumber("Front Right AE Value", frontRightModule.getAbsoluteEncoderDeg(Constants.DriveConstants.kFRDegrees));
            //   //RE Degrees Reading
            //     SmartDashboard.putNumber("Back left RE Value", backLeftModule.getSteerPosition());
            //     SmartDashboard.putNumber("Back Right RE Value", backRightModule.getSteerPosition());
            //     SmartDashboard.putNumber("Front left RE Value", frontLeftModule.getSteerPosition());
            //     SmartDashboard.putNumber("Front Right RE Value", frontRightModule.getSteerPosition());
            //  //RE Distance Reading
            //    SmartDashboard.putNumber("Front Left Drive Position", frontLeftModule.getDrivePosition());
            //    SmartDashboard.putNumber("Front Right Drive Position", frontRightModule.getDrivePosition());
            //    SmartDashboard.putNumber("Back Left Drive Position", backLeftModule.getDrivePosition());
            //    SmartDashboard.putNumber("Back Right Drive Position", backRightModule.getDrivePosition());
            
        }
}
