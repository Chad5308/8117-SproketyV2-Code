package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        shooterSubsystem.setDesiredVelocities(60, 60);
        pitchSubsystem.setPosition(Constants.ShooterConstants.closeSpeakerAngle);
    }

    // public SequentialCommandGroup autoSpeaker(){
    //     return new SequentialCommandGroup(Commands.run(() -> {shooterSubsystem.setDesiredVelocities(75, 75);}).andThen(
    //         Commands.run(() -> {pitchSubsystem.setPosition(Constants.ShooterConstants.closeSpeakerAngle);})).andThen(
    //             Commands.waitSeconds(1)).andThen(
    //                 Commands.run(() -> { shootSpeedBreach();})).andThen(
    //                     Commands.waitSeconds(1)).andThen(
    //                         shooterSubsystem.stopShooter()).andThen(
    //                             Commands.run(() -> {stopBreach();}))
    //                         );
    // }

    public void launch(){
        shooterSubsystem.setBreachSpeed(1);
    }

    public void stopAll(){
        shooterSubsystem.setBreachSpeed(0);
        shooterSubsystem.setDesiredVelocities(0, 0);
    }



    public void path6Shoot(){
        shooterSubsystem.setDesiredVelocities(75, 75);
        pitchSubsystem.setPosition(Constants.ShooterConstants.path6Angle);
        Commands.waitUntil(shooterSubsystem::areBothShootersUpToSpeed).andThen(Commands.runOnce(() -> {shootSpeedBreach();}));
    }

    public void podiumShot(){
        shooterSubsystem.setDesiredVelocities(75, 75);
        pitchSubsystem.setPosition(Constants.ShooterConstants.podiumSpeakerAngle);
    }

    public void stopBreach(){
        shooterSubsystem.setBreachSpeed(0);
    }

    public void shootSpeedBreach(){
        shooterSubsystem.setBreachSpeed(0.75);
    }

    public void reverseBreach(){
        shooterSubsystem.setBreachSpeed(-0.25);
    }


}
