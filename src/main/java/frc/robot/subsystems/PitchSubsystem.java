package frc.robot.subsystems;


import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
public final AbsoluteEncoder pitchEncoder;

public double desiredPosition; //degrees
public double positionTolerance = 1; //degrees





    public PitchSubsystem(){
        lPitchMotor = new CANSparkMax(Constants.ShooterConstants.lPitchEncoder, MotorType.kBrushless);
        rPitchMotor = new CANSparkMax(Constants.ShooterConstants.rPitchEncoder, MotorType.kBrushless);
        lPitchEncoder = lPitchMotor.getEncoder();
        rPitchEncoder = rPitchMotor.getEncoder();
        pitchEncoder = lPitchMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
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

        rPitchEncoder.setPositionConversionFactor(Constants.ShooterConstants.toDegrees);
        lPitchEncoder.setPositionConversionFactor(Constants.ShooterConstants.toDegrees);
        lPitchMotor.setOpenLoopRampRate(10);
        rPitchMotor.setOpenLoopRampRate(10);
        pitchEncoder.setPositionConversionFactor(Constants.ShooterConstants.toDegrees);
        pitchEncoder.setZeroOffset(Constants.ShooterConstants.pitchOffset);
        // rPitchPID.setSmartMotionMaxVelocity(10, 0);
        // lPitchPID.setSmartMotionMaxVelocity(10, 0);
    }


    public double getPosition(){
        return pitchEncoder.getPosition();
    }

    public boolean atPosition(){
        return (Math.abs(getPosition() - desiredPosition)) <= positionTolerance;
    }




@Override
public void periodic() {
    SmartDashboard.putNumber("Shooter Position", rPitchEncoder.getPosition());
    SmartDashboard.putNumber("Desired POsition", desiredPosition);
    SmartDashboard.putBoolean("At Position", atPosition());
}


}
