package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    
public final CANSparkMax fwLeftMotor;
public final CANSparkMax fwRightMotor;
public final CANSparkMax lPitchMotor;
public final CANSparkMax rPitchMotor;

public final RelativeEncoder fwLeftEncoder;
public final RelativeEncoder fwRightEncoder;
public final RelativeEncoder lPitchEncoder;
public final RelativeEncoder rPitchEncoder;

public final SparkPIDController fwLeftPID;
public final SparkPIDController fwRightPID;
public final SparkPIDController lPitchPID;
public final SparkPIDController rPitchPID;

public final TalonFX leftFlyWheel;
public final TalonFX rightFlyWheel;
private final VelocityVoltage motorVelocity = new VelocityVoltage(0);




public boolean lPitchEncoderReversed;



public ShooterSubsystem(){
leftFlyWheel = new TalonFX(Constants.ShooterConstants.leftFlyWheelNum);
rightFlyWheel = new TalonFX(Constants.ShooterConstants.rightFlyWheelNum);

leftFlyWheel.setControl(new Follower(leftFlyWheel.getDeviceID(), Constants.ShooterConstants.leftFlyWheelInverted));
rightFlyWheel.setControl(new Follower(rightFlyWheel.getDeviceID(), Constants.ShooterConstants.rightFlyWheelInverted));

 var slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.1214;
    slot0Configs.kP = 0.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
leftFlyWheel.getConfigurator().apply(slot0Configs, 0.05);
rightFlyWheel.getConfigurator().apply(slot0Configs, 0.05);
motorVelocity.Slot = 0;

    fwLeftMotor = new CANSparkMax(Constants.ShooterConstants.fwLeftMotorNum, MotorType.kBrushless);
    fwRightMotor = new CANSparkMax(Constants.ShooterConstants.fwRightMotorNum, MotorType.kBrushless);
    lPitchMotor = new CANSparkMax(Constants.ShooterConstants.lPitchEncoder, MotorType.kBrushless);
    rPitchMotor = new CANSparkMax(Constants.ShooterConstants.rPitchEncoder, MotorType.kBrushless);
    


    fwLeftMotor.restoreFactoryDefaults();
    fwRightMotor.restoreFactoryDefaults();

    fwLeftMotor.setInverted(Constants.ShooterConstants.fwLeftInverted);
    fwRightMotor.setInverted(Constants.ShooterConstants.fwRightInverted);
    lPitchMotor.setInverted(Constants.ShooterConstants.lPitchReversed);
    rPitchMotor.setInverted(Constants.ShooterConstants.rPitchReversed);

    fwLeftMotor.setIdleMode(IdleMode.kBrake);
    fwRightMotor.setIdleMode(IdleMode.kBrake);
    lPitchMotor.setIdleMode(IdleMode.kBrake);
    rPitchMotor.setIdleMode(IdleMode.kBrake);

    fwLeftMotor.setOpenLoopRampRate(Constants.ShooterConstants.ramp_rate);
    fwRightMotor.setOpenLoopRampRate(Constants.ShooterConstants.ramp_rate);

    fwLeftEncoder = fwLeftMotor.getEncoder();
    fwRightEncoder = fwRightMotor.getEncoder();
    lPitchEncoder = lPitchMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    rPitchEncoder = rPitchMotor.getEncoder();

    fwLeftPID = fwLeftMotor.getPIDController();
    fwRightPID = fwRightMotor.getPIDController();
    lPitchPID = lPitchMotor.getPIDController();
    rPitchPID = rPitchMotor.getPIDController();
    

    

    fwLeftPID.setP(Constants.ShooterConstants.kP_Shooter);
    fwLeftPID.setI(Constants.ShooterConstants.kI_Shooter);
    fwLeftPID.setD(Constants.ShooterConstants.kD_Shooter);

    fwRightPID.setP(Constants.ShooterConstants.kP_Shooter);
    fwRightPID.setI(Constants.ShooterConstants.kI_Shooter);
    fwRightPID.setD(Constants.ShooterConstants.kD_Shooter);

    lPitchPID.setP(Constants.ShooterConstants.kP_pitch);
    lPitchPID.setI(Constants.ShooterConstants.kI_pitch);
    lPitchPID.setD(Constants.ShooterConstants.kD_pitch);

    rPitchPID.setP(Constants.ShooterConstants.kP_pitch);
    rPitchPID.setI(Constants.ShooterConstants.kI_pitch);
    rPitchPID.setD(Constants.ShooterConstants.kD_pitch);

    rPitchEncoder.setPositionConversionFactor(Constants.ShooterConstants.toDegrees);
    lPitchEncoder.setPositionConversionFactor(Constants.ShooterConstants.toDegrees);
    lPitchMotor.setOpenLoopRampRate(10);
    rPitchMotor.setOpenLoopRampRate(10);
rPitchPID.setSmartMotionMaxVelocity(10, 0);
lPitchPID.setSmartMotionMaxVelocity(10, 0);
}


//Shooter Speed methods
public double shootSpeed = 0;
public void setShooterSpeed(double speed){
    // speed = speed <=0 ? 0 : speed;
    fwLeftMotor.set(speed);
    fwRightMotor.set(speed);
    leftFlyWheel.set(speed);
    rightFlyWheel.set(-speed);
}

public Command upSpeedCommand(){return runOnce(() -> { 
    shootSpeed+=0.1; 
    fwLeftMotor.set(shootSpeed); 
    fwRightMotor.set(shootSpeed);
});}

public Command lowerSpeedCommand(){return runOnce(() -> { 
    shootSpeed-=0.1;
    fwLeftMotor.set(shootSpeed);
    fwRightMotor.set(shootSpeed);
});}
public Command stopFWCommand(){return runOnce(() -> { 
    setShooterSpeed(0); 
});}
public Command rampUpCommand(){return runOnce(() -> {
    leftFlyWheel.set(0.7);
    rightFlyWheel.set(-0.85);
});}
public Command fireCommand(){return runOnce(()-> {
    fwLeftMotor.set(0.5);
    fwRightMotor.set(0.85);
    leftFlyWheel.set(0.5);
    rightFlyWheel.set(-0.85);
});}
public Command indexShooterCommand(){
    return runOnce(()-> {
    fwLeftMotor.set(0.1);
    fwRightMotor.set(0.1);
    leftFlyWheel.set(0);
    rightFlyWheel.set(0);
    });
}
public Command reverseIndexShooterCommand(){
    return runOnce(()-> {
    fwLeftMotor.set(-0.1);
    fwRightMotor.set(-0.1);
    leftFlyWheel.set(0);
    rightFlyWheel.set(0);
    });
}

public Command sourceIntakeCommand(){
    return runOnce(() -> {
        setShooterSpeed(-0.5);
    });
}


//Angular methods
// "0" degrees is the home position where a game piece would be indexed

public void setAngle(double angle){ lPitchPID.setReference(angle, ControlType.kPosition); rPitchPID.setReference(angle, ControlType.kPosition);}
public void setSpeed(double speed){lPitchMotor.set(speed); rPitchMotor.set(speed);}
public Command testCommand(){       return runOnce(() -> {  setAngle(90);   });}
public Command pitchStopCommand(){  return runOnce(() -> {  setSpeed(0);});}


public Command rotateOutCommand(){  return runOnce(() -> {  setSpeed(1);});}
public Command rotateInCommand(){  return runOnce(() -> {  setSpeed(-1);});}



//Combined methods

public Command closeSpeakerCommand(){   return runOnce(() -> {  
    setAngle(Constants.ShooterConstants.closeSpeakerAngle);}
);}




@Override
public void periodic() {

    SmartDashboard.putNumber("Shooter Position", rPitchEncoder.getPosition());
    
}

}
