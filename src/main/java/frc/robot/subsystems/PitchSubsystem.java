package frc.robot.subsystems;


import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PitchSubsystem extends SubsystemBase{
    //Pitch Motors
public final CANSparkMax lPitchMotor;
public final CANSparkMax rPitchMotor;
public final RelativeEncoder lPitchEncoder;
public final RelativeEncoder rPitchEncoder;
public final SparkPIDController lPitchPID;
public final SparkPIDController rPitchPID;
public final DigitalInput encoderInput = new DigitalInput(6);
public final DutyCycleEncoder pitchEncoder = new DutyCycleEncoder(encoderInput);

public double desiredPosition; //degrees
public double positionTolerance = 1; //degrees





    public PitchSubsystem(){
        lPitchMotor = new CANSparkMax(Constants.ShooterConstants.lPitchEncoderNum, MotorType.kBrushless);
        rPitchMotor = new CANSparkMax(Constants.ShooterConstants.rPitchEncoderNum, MotorType.kBrushless);
        lPitchEncoder = lPitchMotor.getEncoder();
        rPitchEncoder = rPitchMotor.getEncoder();
        lPitchPID = lPitchMotor.getPIDController();
        rPitchPID = rPitchMotor.getPIDController();
        configure();
    }

    public void configure(){
    //Pitch Config
        lPitchMotor.setInverted(Constants.ShooterConstants.lPitchReversed);
        rPitchMotor.setInverted(Constants.ShooterConstants.rPitchReversed);
        lPitchMotor.setIdleMode(IdleMode.kBrake);
        rPitchMotor.setIdleMode(IdleMode.kBrake);
        lPitchPID.setP(Constants.ShooterConstants.kP_pitch);
        lPitchPID.setI(Constants.ShooterConstants.kI_pitch);
        lPitchPID.setD(Constants.ShooterConstants.kD_pitch);

        rPitchPID.setP(Constants.ShooterConstants.kP_pitch);
        rPitchPID.setI(Constants.ShooterConstants.kI_pitch);
        rPitchPID.setD(Constants.ShooterConstants.kD_pitch);

        rPitchEncoder.setPositionConversionFactor(14.539);
        lPitchEncoder.setPositionConversionFactor(14.539);
        lPitchMotor.setOpenLoopRampRate(10);
        rPitchMotor.setOpenLoopRampRate(10);
        // pitchEncoder.setPositionOffset(Constants.ShooterConstants.pitchOffset);
        // pitchEncoder.setDistancePerRotation(Constants.ShooterConstants.toDegrees);
        // rPitchPID.setSmartMotionMaxVelocity(10, 0);
        // lPitchPID.setSmartMotionMaxVelocity(10, 0);
        
        lPitchEncoder.setPosition(0);
        rPitchEncoder.setPosition(0);
    }


    public double getPosition(){
        return pitchEncoder.getAbsolutePosition();
    }


    public void rotatePositive(){
        lPitchMotor.set(0.5);
        rPitchMotor.set(0.5);
    }

    public void rotateNegative(){
        lPitchMotor.set(-0.15);
        rPitchMotor.set(-0.15);
    }

    public void stopRotation(){
        lPitchMotor.set(0);
        rPitchMotor.set(0);
    }

    public void setPosition(double degree){
        lPitchPID.setReference(degree, com.revrobotics.CANSparkBase.ControlType.kPosition);
        rPitchPID.setReference(degree, com.revrobotics.CANSparkBase.ControlType.kPosition);        
    }

    public void homePosition() {
        lPitchPID.setReference(0, com.revrobotics.CANSparkBase.ControlType.kPosition);
        rPitchPID.setReference(0, com.revrobotics.CANSparkBase.ControlType.kPosition);
    }

    public void setBreach(){
        setPosition(-10.65);
    }

    public void autoSet(){
    // lPitchEncoder.setPosition(getPosition());
    // rPitchEncoder.setPosition(getPosition());
    lPitchPID.setReference(0, ControlType.kPosition);
    rPitchPID.setReference(0, ControlType.kPosition);
    }




@Override
public void periodic() {
    SmartDashboard.putNumber("Shooter Position", getPosition());
    SmartDashboard.putNumber("Shooter test", lPitchEncoder.getPosition());
}


}
