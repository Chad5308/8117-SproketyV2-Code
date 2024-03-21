package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PitchSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootingCommand extends Command{
    public ShooterSubsystem shooterSubsystem;
    public PitchSubsystem pitchSubsystem;
    
    public ShootingCommand(ShooterSubsystem shooterSubsystem, PitchSubsystem pitchSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        this.pitchSubsystem = pitchSubsystem;
        addRequirements(shooterSubsystem, pitchSubsystem);
    }

    @Override
    public void initialize(){
        pitchSubsystem.autoSet();
    }


    @Override
    public void execute(){
        
    }

    public void closeSpeaker(){
        shooterSubsystem.setDesiredVelocities(75, 75);
        pitchSubsystem.setPosition(Constants.ShooterConstants.closeSpeakerAngle);
    }


}
