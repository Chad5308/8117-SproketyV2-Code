package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
public final CANSparkMax rPitchMotor;
public final RelativeEncoder rPitchEncoder;
public final SparkPIDController rPitchPID;
public final DigitalInput encoderInput = new DigitalInput(6);
public final DutyCycleEncoder pitchEncoder;

public double desiredPosition; //degrees
public double positionTolerance = 1; //degrees





    public PitchSubsystem(){
        rPitchMotor = new CANSparkMax(Constants.ShooterConstants.rPitchEncoderNum, MotorType.kBrushless);
        rPitchEncoder = rPitchMotor.getEncoder();
        rPitchPID = rPitchMotor.getPIDController();
        pitchEncoder = new DutyCycleEncoder(encoderInput);
        configure();
    }

    public void configure(){
    //Pitch Config
        rPitchMotor.setInverted(Constants.ShooterConstants.rPitchReversed);
        rPitchMotor.setIdleMode(IdleMode.kBrake);

        rPitchPID.setP(Constants.ShooterConstants.kP_pitch);
        rPitchPID.setI(Constants.ShooterConstants.kI_pitch);
        rPitchPID.setD(Constants.ShooterConstants.kD_pitch);

        rPitchEncoder.setPositionConversionFactor(0.01 * 360);
        rPitchMotor.setOpenLoopRampRate(10);
        rPitchPID.setSmartMotionMaxVelocity(10, 0);
        rPitchEncoder.setPosition(0);
        pitchEncoder.setPositionOffset(0.312);
    }

    
    public double getPosition(){
    double angle = pitchEncoder.getAbsolutePosition() - pitchEncoder.getPositionOffset();
    return angle;
    }


    public void rotatePositive(){
        rPitchMotor.set(1);
    }

    public void rotateNegative(){
        rPitchMotor.set(-1);
    }

    public void stopRotation(){
        rPitchMotor.set(0);
    }

    public void setPosition(double degree){
        rPitchPID.setReference(degree, com.revrobotics.CANSparkBase.ControlType.kPosition);        
    }

    public void homePosition() {
        rPitchPID.setReference(0, com.revrobotics.CANSparkBase.ControlType.kPosition);
    }

    public void autoSet(){
    rPitchEncoder.setPosition(getPosition());
    rPitchPID.setReference(0, ControlType.kPosition);
    }

@Override
public void periodic() {
    getPosition();

    SmartDashboard.putNumber("Shooter Position", getPosition());
    SmartDashboard.putNumber("Shooter test", rPitchEncoder.getPosition());
    SmartDashboard.putNumber("ABS reading", pitchEncoder.getAbsolutePosition());
}


}
