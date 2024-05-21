package frc.robot.commands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivebase.LimelightSubsystem;
import frc.robot.subsystems.Drivebase.SwerveSubsystem;


public class DriveCommand extends Command{
    

    
    private final SwerveSubsystem swerveSubsystem;
    public final LimelightSubsystem LL_Sub;
    public final CommandXboxController opController;
    public final CommandJoystick leftStick;
    public final CommandJoystick rightStick;
    public boolean controllerType = true;
    public final RobotContainer robotContainer;

    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean fieldOriented=false;
     public double ySpeed, xSpeed, turningSpeed;
     public double ll_zSpeed, ll_xSpeed, ll_turningSpeed;
    public boolean autoAlign = false;
    public ChassisSpeeds chassisSpeeds;

    



    public DriveCommand(SwerveSubsystem swerveSubsystem, LimelightSubsystem LL_Sub, CommandXboxController opController, CommandJoystick leftStick, CommandJoystick rightStick, RobotContainer robotContainer) {
                this.swerveSubsystem = swerveSubsystem;
                this.LL_Sub = LL_Sub;
                this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.turningLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
                addRequirements(swerveSubsystem, LL_Sub);
                this.opController = opController;
                this.leftStick = leftStick;
                this.rightStick = rightStick;
                this.robotContainer = robotContainer;
    }


    @Override
    public void initialize() {
     swerveSubsystem.faceAllFoward();
     LL_Sub.setThetaPID();
     LL_Sub.setLinearPID();
    }

 


    @Override
    public void execute() {
        // controllerType = robotContainer.controllerType;
      
        xSpeed = controllerType? -leftStick.getX(): -opController.getLeftX();
        ySpeed = controllerType? -leftStick.getY(): -opController.getLeftY();
        turningSpeed = controllerType? -rightStick.getX(): -opController.getRightX();
        
       

        fieldOriented = swerveSubsystem.fieldOriented;

        ll_xSpeed = LL_Sub.xSpeed;
        ll_zSpeed = LL_Sub.zSpeed;
        ll_turningSpeed = LL_Sub.autoAlign();

        
        SmartDashboard.putBoolean("auto align", autoAlign);
        SmartDashboard.putBoolean("fieldOriented", fieldOriented);
        SmartDashboard.putNumber("AutoAlign", ll_turningSpeed);

        SmartDashboard.putNumber("Left stick", leftStick.getX());
        SmartDashboard.putNumber("X Speed", xSpeed);
        SmartDashboard.putBoolean("Controller type", controllerType);


        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds;
        if (autoAlign){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed * -1, xSpeed * -1, ll_turningSpeed, swerveSubsystem.geRotation2d());
        } else if(fieldOriented){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, turningSpeed, swerveSubsystem.geRotation2d());
        }else {
            chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);
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
