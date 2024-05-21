package frc.robot.commands;

import com.fasterxml.jackson.core.sym.Name;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PitchSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Drivebase.LimelightSubsystem;
import frc.robot.subsystems.Drivebase.SwerveSubsystem;

public class AutoCommand {
    
public ShootingCommand shootingCommand;
public TransferCommand transferCommand;
public DriveCommand driveCommand;
public SwerveSubsystem swerveSubsystem;
public IntakeSubsystem intakeSubsystem;
public ShooterSubsystem shooter_sub;
public PitchSubsystem pitchSubsystem;
public LimelightSubsystem limelightSubsystem;
public PIDController translationConstants = new PIDController(Constants.AutoConstants.kPTranslation, Constants.AutoConstants.kITranslation, Constants.AutoConstants.kDTranslation);
public PIDController rotationConstants = new PIDController(Constants.AutoConstants.kPTheta, Constants.AutoConstants.kITheta, Constants.AutoConstants.kDTheta);





    public AutoCommand(ShootingCommand shootingCommand, TransferCommand transferCommand, DriveCommand driveCommand, SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooter_sub, PitchSubsystem pitchSubsystem, LimelightSubsystem limelightSubsystem){
        this.shootingCommand = shootingCommand;
        this.transferCommand = transferCommand;
        this.driveCommand = driveCommand;
        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.pitchSubsystem = pitchSubsystem;
        this.shooter_sub = shooter_sub;
        this.limelightSubsystem = limelightSubsystem;
        // translationConstants.setTolerance(0.1);//meters
        // rotationConstants.setTolerance(10); //maybe degrees?

        AutoBuilder.configureHolonomic(
                swerveSubsystem::getPose, 
                swerveSubsystem::resetOdometry, 
                swerveSubsystem::getRobotRelativeSpeeds, 
                swerveSubsystem::driveRobotRelative, 
                autoConfig, 
                swerveSubsystem::allianceCheck,
                swerveSubsystem
                );
                
                
                NamedCommands.registerCommand("FaceForward Wheels", Commands.runOnce(() -> swerveSubsystem.faceAllFoward()));
                NamedCommands.registerCommand("Intake", intakeSubsystem.runIntakeCommand().andThen(intakeSubsystem.dropIntakeCommand()));
                NamedCommands.registerCommand("revSpeaker", Commands.runOnce(() -> shootingCommand.closeSpeaker()));
                NamedCommands.registerCommand("Launch", Commands.runOnce(() -> {shootingCommand.launch();}));
                NamedCommands.registerCommand("stopEverything", Commands.runOnce(() -> {shootingCommand.stopAll();}));
                NamedCommands.registerCommand("stopIntake", intakeSubsystem.stopIntakeCommand());
                NamedCommands.registerCommand("homeShooter", Commands.runOnce(() -> {pitchSubsystem.autoSet();}));
                NamedCommands.registerCommand("stopTransfer", intakeSubsystem.stopIntakeCommand());
                NamedCommands.registerCommand("revPath2", Commands.runOnce(()->{shootingCommand.path2Shoot();}));
                NamedCommands.registerCommand("breachTransfer", Commands.runOnce(()->{shooter_sub.setBreachSpeed(0.25);}));
                NamedCommands.registerCommand("dropIntake", intakeSubsystem.dropIntakeCommand());
                NamedCommands.registerCommand("AutoAim", Commands.runOnce(()-> {limelightSubsystem.autoAngle();}));
                NamedCommands.registerCommand("AutoAlignSpeaker", Commands.runOnce(()->{limelightSubsystem.autoAlign();}));
    }


    public HolonomicPathFollowerConfig autoConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(translationConstants.getP(), translationConstants.getI(), translationConstants.getD()),
        new PIDConstants(rotationConstants.getP(), rotationConstants.getI(), rotationConstants.getD()), 
        Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, 
        Constants.ModuleConstants.moduleRadius, 
        new ReplanningConfig());
    

    
    
    
}
