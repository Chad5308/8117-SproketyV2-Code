package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PitchSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Ultrasonic;

public class TransferCommand extends Command{
    
    public final ShooterSubsystem shooter_sub;
    public final IntakeSubsystem intakeSubsystem;
    public final PitchSubsystem pitchSubsystem;
    // public Ultrasonic intakeSensor = new Ultrasonic(0,1);
    // public Ultrasonic shooterSensor = new Ultrasonic(2, 3);


    public TransferCommand(ShooterSubsystem shooter_sub, IntakeSubsystem intakeSubsystem, PitchSubsystem pitchSubsystem) {
        this.shooter_sub = shooter_sub;
        this.intakeSubsystem = intakeSubsystem;
        this.pitchSubsystem = pitchSubsystem;
    }


    @Override
    public void initialize(){
      
    }

    @Override
    public void execute(){
        if(!intakeSubsystem.isRetracted){
        //    if()
        }
    }
}
