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
private final CANSparkMax rPitchMotor;
private final RelativeEncoder rPitchEncoder;
private final SparkPIDController rPitchPID;
public final DigitalInput encoderInput = new DigitalInput(6);
private final DutyCycleEncoder pitchEncoder;

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

        rPitchEncoder.setPositionConversionFactor(0.03333 * 360);
        rPitchMotor.setOpenLoopRampRate(5);
        rPitchPID.setSmartMotionMaxVelocity(5, 0);
        rPitchEncoder.setPosition(0);
        pitchEncoder.setPositionOffset(0.4693);//0.4669
    }

    
    public double getPosition(){
    double angle = pitchEncoder.getAbsolutePosition() - pitchEncoder.getPositionOffset();
    return angle*360;
    }


    public void rotatePositive(){
        setPosition(rPitchEncoder.getPosition() + 10);
    }

    public void rotateNegative(){
        setPosition(rPitchEncoder.getPosition() - 10);
    }

    public void setPosition(double degree){
        rPitchPID.setReference(degree, com.revrobotics.CANSparkBase.ControlType.kPosition);        
    }

    public void autoSet(){
    rPitchEncoder.setPosition(getPosition());
    rPitchPID.setReference(0, ControlType.kPosition);
    }

    public void gravity(){
        rPitchPID.setP((0.002 * Math.sin(Math.toRadians(rPitchEncoder.getPosition()))) + Constants.ShooterConstants.kP_pitch);
    }

@Override
public void periodic() {
    getPosition();
    rPitchEncoder.setPosition(getPosition());


    SmartDashboard.putNumber("Shooter Position", getPosition());
    SmartDashboard.putNumber("Shooter test", rPitchEncoder.getPosition());
    // SmartDashboard.putNumber("ABS reading", pitchEncoder.getAbsolutePosition());
}


}
