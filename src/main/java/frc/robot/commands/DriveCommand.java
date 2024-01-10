package frc.robot.commands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCommand extends Command{
    

    
    private final SwerveSubsystem swerveSubsystem;
    public final CommandXboxController opController;
    // private final Joystick driveStick;
    // private final Joystick thetaStick;

    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean fieldOriented=false;
     public double ySpeed;
     public double xSpeed;
     public double turningSpeed;
     public CameraServer frontCamera;



    public DriveCommand(SwerveSubsystem swerveSubsystem, CommandXboxController opController) {
                this.swerveSubsystem = swerveSubsystem;
                this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.turningLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
                addRequirements(swerveSubsystem);
                // thetaStick = new Joystick(3);
                // driveStick = new Joystick(0);
                this.opController = opController;

            CameraServer.startAutomaticCapture();
            

    }


    @Override
    public void initialize() {
     swerveSubsystem.faceAllFoward();


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
        turningSpeed = opController.getRightX();//1.85
        SmartDashboard.putBoolean("fieldOriented", fieldOriented);
        fieldOriented = swerveSubsystem.fieldOriented;

        

       
//flight stick init and debugging code. Alt drive method
    //     xSpeed = driveStick.getX()*-1;
    //     ySpeed = driveStick.getY()*-1;
    //     turningSpeed = thetaStick.getX()*-1;
    //    SmartDashboard.putNumber("Left Stick X", driveStick.getX());
    //     SmartDashboard.putNumber("Left Stick Y", driveStick.getY());
    //     SmartDashboard.putNumber("turningSpeed", turningSpeed);
        // if(driveStick.getRawButtonPressed(4)) {
        //         swerveSubsystem.zeroHeading();
        // }
     
        // if(driveStick.getRawButtonPressed(3)) {
//     fieldOriented = swerveSubsystem.fieldOriented;
// }


        // 2. Apply deadband
         xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
         ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
         turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // // 2. Apply deadband again
         xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
         ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
         turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOriented) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed * -1, xSpeed * -1, turningSpeed, swerveSubsystem.geRotation2d());
        } else {
            // Relative to robot. 
            chassisSpeeds = new ChassisSpeeds(ySpeed * -1, xSpeed * -1, turningSpeed);
        }
        // // 6. Output each module states to wheels
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
