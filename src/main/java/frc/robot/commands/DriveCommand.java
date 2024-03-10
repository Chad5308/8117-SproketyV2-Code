package frc.robot.commands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivebase.LimelightSubsystem;
import frc.robot.subsystems.Drivebase.SwerveSubsystem;


public class DriveCommand extends Command{
    

    
    private final SwerveSubsystem swerveSubsystem;
    public final LimelightSubsystem LL_Sub;
    public final CommandXboxController opController;

    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean fieldOriented=false;
     public double ySpeed, xSpeed, turningSpeed;
     public double ll_zSpeed, ll_xSpeed, ll_turningSpeed;
    public boolean limelightTracking = false;
    public CameraServer camera;
    



    public DriveCommand(SwerveSubsystem swerveSubsystem, LimelightSubsystem LL_Sub, CommandXboxController opController) {
                this.swerveSubsystem = swerveSubsystem;
                this.LL_Sub = LL_Sub;
                this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.turningLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
                addRequirements(swerveSubsystem, LL_Sub);
                this.opController = opController;

            CameraServer.startAutomaticCapture();

    }


    @Override
    public void initialize() {
     swerveSubsystem.faceAllFoward();
     LL_Sub.setThetaPID();
     LL_Sub.setLinearPID();
    }

    //For pathplanner setup
    public ChassisSpeeds AutoSpeeds = new ChassisSpeeds(ySpeed * -1, xSpeed * -1, turningSpeed);
    public ChassisSpeeds getSpeeds(){
        return AutoSpeeds;
    }
   


    @Override
    public void execute() {

 
        

//Xbox joystick init and debugging code. Main drive method
        xSpeed = opController.getLeftX() / 1;//1.35
        ySpeed = opController.getLeftY() / 1;//1.35
        turningSpeed = -opController.getRightX();//1.85
        fieldOriented = swerveSubsystem.fieldOriented;

        ll_xSpeed = LL_Sub.xSpeed;
        ll_zSpeed = LL_Sub.zSpeed;
        ll_turningSpeed = LL_Sub.turningSpeed;
        limelightTracking = LL_Sub.autoAlign;


        SmartDashboard.putBoolean("fieldOriented", fieldOriented);

        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;


        ChassisSpeeds chassisSpeeds;
        if (limelightTracking){
            chassisSpeeds = new ChassisSpeeds(-ll_zSpeed, -ll_xSpeed, ll_turningSpeed);
        } else if(fieldOriented){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed * -1, xSpeed * -1, turningSpeed, swerveSubsystem.geRotation2d());
        }else {
            chassisSpeeds = new ChassisSpeeds(ySpeed * -1, xSpeed * -1, turningSpeed);
        }


        swerveSubsystem.setModuleStates(chassisSpeeds);
    }


   



    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }





}
