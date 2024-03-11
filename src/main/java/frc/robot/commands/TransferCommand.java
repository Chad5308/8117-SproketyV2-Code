package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class TransferCommand extends Command{
    
    public final ShooterSubsystem shooter_sub;

    public TransferCommand(ShooterSubsystem shooter_sub) {
        this.shooter_sub = shooter_sub;
    }


    @Override
    public void initialize(){
      
    }
}
