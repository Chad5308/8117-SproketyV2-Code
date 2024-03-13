package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootingCommand extends Command{
    public ShooterSubsystem shooterSubsystem;
    
    public ShootingCommand(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
    }


    @Override
    public void execute(){
        
    }


}
