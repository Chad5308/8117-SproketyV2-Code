package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    
//Flywheels
public final TalonFX topMotor;
public final TalonFX bottomMotor;
public VelocityVoltage velocityRequest;
 MotionMagicVelocityVoltage motionMagicRequest;
 TalonFXConfiguration topConfig, bottomConfig;

//Unit in Rotations per second
double desiredTopVelocity = 0;
double desiredBottomVelocity = 0;

double shooterUpToSpeedTolerance = 0.7;


public ShooterSubsystem(){
    topMotor = new TalonFX(Constants.ShooterConstants.topMotorNum, "rio");
    bottomMotor = new TalonFX(Constants.ShooterConstants.bottomMotorNum, "rio");

    topConfig = new TalonFXConfiguration();
    bottomConfig = new TalonFXConfiguration();

    velocityRequest = new VelocityVoltage(0).withSlot(0);
    motionMagicRequest = new MotionMagicVelocityVoltage(0);
        
    configure();
}

public void configure(){
    topConfig.Slot0.kS = Constants.ShooterConstants.kS_TopShooter;
    topConfig.Slot0.kV = Constants.ShooterConstants.kV_TopShooter;
    topConfig.Slot0.kA = Constants.ShooterConstants.kA_TopShooter;
    topConfig.Slot0.kP = Constants.ShooterConstants.kP_TopShooter;
    topConfig.Slot0.kI = Constants.ShooterConstants.kI_TopShooter;
    topConfig.Slot0.kD = Constants.ShooterConstants.kD_TopShooter;

    topConfig.MotionMagic.MotionMagicAcceleration = 400;
    topConfig.MotionMagic.MotionMagicJerk = 4000;

    bottomConfig.Slot0.kS = Constants.ShooterConstants.kS_BottomShooter;
    bottomConfig.Slot0.kV = Constants.ShooterConstants.kV_BottomShooter;
    bottomConfig.Slot0.kA = Constants.ShooterConstants.kA_BottomShooter;
    bottomConfig.Slot0.kP = Constants.ShooterConstants.kP_BottomShooter;
    bottomConfig.Slot0.kI = Constants.ShooterConstants.kI_BottomShooter;
    bottomConfig.Slot0.kD = Constants.ShooterConstants.kD_BottomShooter;

    bottomConfig.MotionMagic.MotionMagicAcceleration = 400;
    bottomConfig.MotionMagic.MotionMagicJerk = 4000;

    topMotor.getConfigurator().apply(topConfig);
    bottomMotor.getConfigurator().apply(bottomConfig);

    topMotor.setInverted(Constants.ShooterConstants.topMotorReversed);
    bottomMotor.setInverted(Constants.ShooterConstants.bottomMotorReversed);

}


public void getUpToSpeed() {
    if (desiredTopVelocity <= 0 && desiredBottomVelocity <= 0) {
      setShootingNeutralOutput();
    } else {
      topMotor.setControl(motionMagicRequest.withVelocity(desiredTopVelocity));
      bottomMotor.setControl(motionMagicRequest.withVelocity(desiredBottomVelocity));
    }
  }

public void setShootingNeutralOutput() {
    topMotor.setControl(new NeutralOut());
    bottomMotor.setControl(new NeutralOut());
}

public double getTopShooterVelocity() {
    return topMotor.getVelocity().getValueAsDouble();
}
public double getBottomShooterVelocity() {
    return bottomMotor.getVelocity().getValueAsDouble();
}
public boolean isTopShooterUpToSpeed() {
    return (Math.abs(getTopShooterVelocity() - desiredTopVelocity)) <= shooterUpToSpeedTolerance;
}
public boolean isBotomShooterUpToSpeed() {
    return (Math.abs(getBottomShooterVelocity() - desiredBottomVelocity)) <= shooterUpToSpeedTolerance;
}
public boolean areBothShootersUpToSpeed() {
    return isTopShooterUpToSpeed()
        && isBotomShooterUpToSpeed();
}
public void setTopDesiredVelocity(double desiredVelocity) {
    desiredTopVelocity = desiredVelocity;
}
public void setBottomDesiredVelocity(double desiredVelocity) {
    desiredBottomVelocity = desiredVelocity;
}
public void setDesiredVelocities(double desiredLeftVelocity, double desiredRightVelocity) {
    setTopDesiredVelocity(desiredLeftVelocity);
    setBottomDesiredVelocity(desiredRightVelocity);
}



public Command speedUpCommand(){
    return runOnce(() -> {
            desiredBottomVelocity = 60;
            desiredTopVelocity = 60;
    });
}

public Command slowShooter(){
    return runOnce(() -> {
        desiredBottomVelocity -= 5;
        desiredTopVelocity -=5;
    });
}

public Command stopShooter(){
    return runOnce(() -> {
        desiredBottomVelocity = 0;
        desiredTopVelocity = 0;
    });
}

public Command fastShooter(){
    return runOnce(() -> {
        desiredBottomVelocity += 5;
        desiredTopVelocity +=5;
    });
}

@Override
public void periodic() {
    getUpToSpeed();

    SmartDashboard.putNumber("Shooter/Top/Velocity RPS", getTopShooterVelocity());
    SmartDashboard.putNumber("Shooter/Top/Desired Velocity RPS", desiredTopVelocity);
    SmartDashboard.putBoolean("Shooter/Top/Up to Speed", isTopShooterUpToSpeed());

    SmartDashboard.putNumber("Shooter/Bottom/Velocity RPS", getBottomShooterVelocity());
    SmartDashboard.putNumber("Shooter/Bottom/Desired Velocity RPS", desiredBottomVelocity);
    SmartDashboard.putBoolean("Shooter/Bottom/Up to Speed", isBotomShooterUpToSpeed());
    
}

}
