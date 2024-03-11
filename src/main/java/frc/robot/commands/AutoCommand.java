package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase.SwerveSubsystem;

public class AutoCommand {
    
public ShootingCommand shootingCommand;
public TransferCommand transferCommand;
public DriveCommand driveCommand;
public SwerveSubsystem swerveSubsystem;





    public AutoCommand(ShootingCommand shootingCommand, TransferCommand transferCommand, DriveCommand driveCommand, SwerveSubsystem swerveSubsystem){
        this.shootingCommand = shootingCommand;
        this.transferCommand = transferCommand;
        this.driveCommand = driveCommand;
        this.swerveSubsystem = swerveSubsystem;


        AutoBuilder.configureHolonomic(
                swerveSubsystem::getPose, 
                swerveSubsystem::resetOdometry, 
                swerveSubsystem::getRobotRelativeSpeeds, 
                swerveSubsystem::setModuleStates, 
                new HolonomicPathFollowerConfig(new PIDConstants(Constants.AutoConstants.kPTranslation, Constants.AutoConstants.kITranslation, Constants.AutoConstants.kDTranslation), new PIDConstants(Constants.AutoConstants.kITheta, Constants.AutoConstants.kITheta, Constants.AutoConstants.kDTheta), 
                Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, 
                Constants.ModuleConstants.moduleRadius, 
                new ReplanningConfig(false, false)), 
                swerveSubsystem::allianceCheck,
                swerveSubsystem
                );
                
                
                NamedCommands.registerCommand("FaceForward Wheels", Commands.runOnce(() -> swerveSubsystem.faceAllFoward()));
    }
    
    
    
    
}
