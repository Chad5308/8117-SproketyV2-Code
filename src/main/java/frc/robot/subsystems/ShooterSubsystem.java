package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
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
 NeutralOut neutralOut;
public final CANSparkMax breachMotor;
public final RelativeEncoder breachEncoder;
public final SparkPIDController breachPID;
public DigitalInput shooterSensor;

//Unit in Rotations per second
double desiredTopVelocity = 0;
double desiredBottomVelocity = 0;

double shooterUpToSpeedTolerance = 5;


public ShooterSubsystem(){
    topMotor = new TalonFX(Constants.ShooterConstants.topMotorNum, "rio");
    bottomMotor = new TalonFX(Constants.ShooterConstants.bottomMotorNum, "rio");
    breachMotor = new CANSparkMax(Constants.ShooterConstants.breachMotorNum, MotorType.kBrushless);
    breachEncoder = breachMotor.getEncoder();
    breachPID = breachMotor.getPIDController();

    topConfig = new TalonFXConfiguration();
    bottomConfig = new TalonFXConfiguration();

    velocityRequest = new VelocityVoltage(0).withSlot(0);
    motionMagicRequest = new MotionMagicVelocityVoltage(0);
        
    shooterSensor = new DigitalInput(3);
    configure();
}

public void configure(){
    //Flywheel config
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

    //Breach Motor Config
    breachMotor.restoreFactoryDefaults();
    breachPID.setP(Constants.ShooterConstants.kP_breach);
    breachPID.setI(Constants.ShooterConstants.kI_breach);
    breachPID.setD(Constants.ShooterConstants.kD_breach);
    breachMotor.setInverted(Constants.ShooterConstants.breachReversed);
    breachMotor.setOpenLoopRampRate(Constants.ShooterConstants.ramp_rate);
    breachMotor.setIdleMode(IdleMode.kCoast);

    neutralOut = new NeutralOut();
}


public void getUpToSpeed() {
    if (desiredTopVelocity <= 0 && desiredBottomVelocity <= 0) {
      setShootingNeutralOutput();
    } else {
      topMotor.setControl(motionMagicRequest.withVelocity(desiredTopVelocity));
      bottomMotor.setControl(motionMagicRequest.withVelocity(desiredBottomVelocity));
    // topMotor.set(desiredTopVelocity);
    // bottomMotor.set(desiredBottomVelocity);
    }
  }

public void setShootingNeutralOutput() {
    topMotor.setControl(neutralOut);
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

public void setDesiredVelocities(double desiredTopVelocity, double desiredBottomVelocity) {
    this.desiredTopVelocity = desiredTopVelocity;
    this.desiredBottomVelocity = desiredBottomVelocity;
}

public void setBreachSpeed(double speed){
    breachMotor.set(speed);
}


// public boolean isGamePiece(){
//     return shooterSensor.get();
// }

@Override
public void periodic() {
    getUpToSpeed();

    // SmartDashboard.putNumber("Shooter/Top/Velocity RPS", getTopShooterVelocity());
    // SmartDashboard.putNumber("Shooter/Top/Desired Velocity RPS", desiredTopVelocity);
    // SmartDashboard.putBoolean("Shooter/Top/Up to Speed", isTopShooterUpToSpeed());

    // SmartDashboard.putNumber("Shooter/Bottom/Velocity RPS", getBottomShooterVelocity());
    // SmartDashboard.putNumber("Shooter/Bottom/Desired Velocity RPS", desiredBottomVelocity);
    // SmartDashboard.putBoolean("Shooter/Bottom/Up to Speed", isBotomShooterUpToSpeed());


    // SmartDashboard.putBoolean("Shooter sensor", shooterSensor.get());
    
}

}
