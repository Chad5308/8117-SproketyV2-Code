package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightCommand extends Command{
    
public SwerveSubsystem s_Swerve;
public LimelightSubsystem LL_Sub;
public final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
public double xSpeed, ySpeed, turningSpeed, thetaSpeed;




    public LimelightCommand(SwerveSubsystem s_swerve, LimelightSubsystem LL_Sub){
        this.s_Swerve = s_swerve;
        this.LL_Sub = LL_Sub;
        this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(s_swerve, LL_Sub);
    }

   



    @Override
    public void initialize(){
        s_Swerve.faceAllFoward();
        LL_Sub.setPID();
    }

    @Override
    public void execute(){
        ChassisSpeeds limelightSpeeds;
       turningSpeed = LL_Sub.turningSpeed;

        xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        // thetaSpeed = turningLimiter.calculate(turningSpeed) * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;



        limelightSpeeds = new ChassisSpeeds(0, 0, turningSpeed);
        s_Swerve.setModuleStates(limelightSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        // if(LL_Sub.hasTargets.get() == 0){
        //     return true;
        // }else {return false;}

        return false;
    }
}
