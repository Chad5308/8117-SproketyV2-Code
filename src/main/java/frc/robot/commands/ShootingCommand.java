package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.PitchSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Drivebase.LimelightSubsystem;

public class ShootingCommand extends Command{
    public ShooterSubsystem shooterSubsystem;
    public PitchSubsystem pitchSubsystem;
    public LimelightSubsystem limelightSubsystem;
    public boolean isFlipped = false;
    public boolean autoAim = false;
    
    public ShootingCommand(ShooterSubsystem shooterSubsystem, PitchSubsystem pitchSubsystem, LimelightSubsystem limelightSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        this.pitchSubsystem = pitchSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(shooterSubsystem, pitchSubsystem);
    }

    @Override
    public void initialize(){
        pitchSubsystem.autoSet();
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("is flipped", isFlipped);
        if(autoAim){
            pitchSubsystem.setPosition(limelightSubsystem.autoAngle());
        }
    }
    public int isFlipped(){
        int temp = 0;
        return temp = isFlipped ? -1 : 1;
    }
    public void closeSpeaker(){
        shooterSubsystem.setDesiredVelocities(60, 60);
        pitchSubsystem.setPosition(isFlipped() * Constants.ShooterConstants.closeSpeakerAngle);
    }
    public void launch(){
        shooterSubsystem.setBreachSpeed(1);
    }
    public void stopAll(){
        shooterSubsystem.setBreachSpeed(0);
        shooterSubsystem.setDesiredVelocities(0, 0);
    }
    public void path2Shoot(){
        shooterSubsystem.setDesiredVelocities(75, 75);
        pitchSubsystem.setPosition(isFlipped() * Constants.ShooterConstants.path2Angle);
        Commands.waitUntil(shooterSubsystem::areBothShootersUpToSpeed).andThen(Commands.runOnce(() -> {shootSpeedBreach();}));
    }
    public void podiumShot(){
        shooterSubsystem.setDesiredVelocities(75, 75);
        pitchSubsystem.setPosition(isFlipped() * Constants.ShooterConstants.podiumSpeakerAngle);
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
    public void speedUpFlyWheels(){
        shooterSubsystem.setDesiredVelocities(100, 100);
        pitchSubsystem.setPosition(pitchSubsystem.getPosition());
    }
    public void stopFlyWheels(){
        shooterSubsystem.setDesiredVelocities(0, 0);
        pitchSubsystem.setPosition(0);
    }


}
