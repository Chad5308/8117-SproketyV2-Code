package frc.robot.subsystems;

import javax.naming.PartialResultException;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    
public final CANSparkMax fwLeftMotor;
public final CANSparkMax fwRightMotor;
public final CANSparkMax indexerMotor;
public final CANSparkMax pitchMotor;

public final RelativeEncoder fwLeftEncoder;
public final RelativeEncoder fwRightEncoder;
public final RelativeEncoder indexEncoder;
public final AbsoluteEncoder pitchEncoder;

public final SparkPIDController fwLeftPID;
public final SparkPIDController fwRightPID;
public final SparkPIDController indexPID;
public final SparkPIDController pitchPID;

public final DigitalOutput shooterIndexer;

public boolean pitchEncoderReversed;



public ShooterSubsystem(){
    fwLeftMotor = new CANSparkMax(Constants.ShooterConstants.fwLeftMotorNum, MotorType.kBrushless);
    fwRightMotor = new CANSparkMax(Constants.ShooterConstants.fwRightMotorNum, MotorType.kBrushless);
    indexerMotor = new CANSparkMax(Constants.ShooterConstants.indexerMotor, MotorType.kBrushless);
    pitchMotor = new CANSparkMax(Constants.ShooterConstants.pitchEncoder, MotorType.kBrushless);
    pitchEncoderReversed = Constants.ShooterConstants.pitchReversed;
    


    fwLeftMotor.restoreFactoryDefaults();
    fwRightMotor.restoreFactoryDefaults();
    indexerMotor.restoreFactoryDefaults();

    fwLeftMotor.setIdleMode(IdleMode.kCoast);
    fwRightMotor.setIdleMode(IdleMode.kCoast);
    indexerMotor.setIdleMode(IdleMode.kBrake);

    fwLeftMotor.setOpenLoopRampRate(Constants.ShooterConstants.ramp_rate);
    fwRightMotor.setOpenLoopRampRate(Constants.ShooterConstants.ramp_rate);
    indexerMotor.setOpenLoopRampRate(Constants.ShooterConstants.ramp_rate);

    fwLeftEncoder = fwLeftMotor.getEncoder();
    fwRightEncoder = fwRightMotor.getEncoder();
    indexEncoder = indexerMotor.getEncoder();
    pitchEncoder = pitchMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    fwLeftPID = fwLeftMotor.getPIDController();
    fwRightPID = fwRightMotor.getPIDController();
    indexPID = indexerMotor.getPIDController();
    pitchPID = pitchMotor.getPIDController();
    

    fwLeftPID.setP(Constants.ShooterConstants.kP_Shooter);
    fwLeftPID.setI(Constants.ShooterConstants.kI_Shooter);
    fwLeftPID.setD(Constants.ShooterConstants.kD_Shooter);

    fwRightPID.setP(Constants.ShooterConstants.kP_Shooter);
    fwRightPID.setI(Constants.ShooterConstants.kI_Shooter);
    fwRightPID.setD(Constants.ShooterConstants.kD_Shooter);

    indexPID.setP(Constants.ShooterConstants.kP_Index);
    indexPID.setI(Constants.ShooterConstants.kI_Index);
    indexPID.setD(Constants.ShooterConstants.kD_Index);

    pitchPID.setP(Constants.ShooterConstants.kP_pitch);
    pitchPID.setI(Constants.ShooterConstants.kI_pitch);
    pitchPID.setD(Constants.ShooterConstants.kD_pitch);

    pitchEncoder.setPositionConversionFactor(Constants.ShooterConstants.toDegrees);
    pitchEncoder.setZeroOffset(Constants.ShooterConstants.pitchOffset);

    shooterIndexer = new DigitalOutput(Constants.ShooterConstants.indexSensor);
}


//Shooter Speed methods

public void setShooterSpeed(double speed){
    speed = speed <=0 ? 0 : speed;
    fwLeftPID.setReference(speed, ControlType.kVelocity);
    fwRightPID.setReference(speed, ControlType.kVelocity);
}

public double getShooterSpeed(){            return Math.max(fwLeftMotor.get(), fwRightMotor.get());}
public Command upSpeedCommand(){            return runOnce(() -> { setShooterSpeed(getShooterSpeed()+20); });}
public Command lowerSpeedCommand(){         return runOnce(() -> { setShooterSpeed(getShooterSpeed()-20); });}
public Command stopCommand(){               return runOnce(() -> { setShooterSpeed(0); });}
public Command closeSpeakerSpeedCommand(){  return runOnce(() -> { setShooterSpeed(60);});}
public Command podiumSpeakerSpeedCommand(){ return runOnce(() -> { setShooterSpeed(60);});}


//Angular methods
// "0" degrees is the home position where a game piece would be indexed
public double getAngle() {          return pitchEncoder.getPosition();}
public void setAngle(double angle){ pitchPID.setReference(angle, ControlType.kPosition);}
public Command homeCommand(){       return runOnce(() -> {  setAngle(0);    });}
public Command extendCommand(){     return runOnce(() -> {  setAngle(getAngle()+5);    });}
public Command retractCommand(){    return runOnce(() -> {  setAngle(getAngle()-5);    });}
public Command pitchStopCommand(){  return runOnce(() -> {  pitchPID.setReference(0, ControlType.kVelocity);  });}


//Indexer methods
public boolean isPresent(){               return shooterIndexer.get();}
public void setIndexSpeed(double speed){  indexPID.setReference(speed, ControlType.kVelocity);}
public double getIndexSpeed(){            return indexEncoder.getVelocity();}
public Command runIndexMotorCommand(){    return runOnce(() -> {    setIndexSpeed(1);   });}
public Command stopIndexMotorCommand(){   return runOnce(() -> {    setIndexSpeed(0);   });}



//Combined methods
public ParallelCommandGroup resetShooterCommand(){return
    stopIndexMotorCommand().alongWith(
    homeCommand()).alongWith(
    stopCommand()
);}
public ParallelCommandGroup pickupPieceCommand(){return 
    runIndexMotorCommand().alongWith(
    holdPieceCommand()
);}
public ParallelCommandGroup holdPieceCommand(){return
    stopIndexMotorCommand().alongWith(
    runOnce(() -> { setAngle(25);})
);}
public SequentialCommandGroup closeSpeakerCommand(){   return 
    runOnce(() -> {  setAngle(Constants.ShooterConstants.closeSpeakerAngle);}).andThen(
    closeSpeakerSpeedCommand()).andThen(
    Commands.waitSeconds(2)).andThen(
    runIndexMotorCommand()
);}
public SequentialCommandGroup podiumSpeakerCommand(){   return
    runOnce(() -> {  setAngle(Constants.ShooterConstants.podiumSpeakerAngle);}).andThen(
    podiumSpeakerSpeedCommand()).andThen(
    Commands.waitSeconds(2)).andThen(
    runIndexMotorCommand()
);}



@Override
public void periodic() {
    SmartDashboard.putNumber("Speed of Shooter", getShooterSpeed());
    SmartDashboard.putNumber("Shooter Position", getAngle());
    
}

}
