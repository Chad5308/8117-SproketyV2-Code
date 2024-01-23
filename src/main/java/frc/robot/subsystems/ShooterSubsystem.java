package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
public final AbsoluteEncoder shooterAbsoluteEncoder;

public final SparkPIDController fwLeftPID;
public final SparkPIDController fwRightPID;
public final SparkPIDController indexPID;

public final DigitalOutput shooterIndexer;



public ShooterSubsystem(){
    fwLeftMotor = new CANSparkMax(Constants.ShooterConstants.fwLeftMotorNum, MotorType.kBrushless);
    fwRightMotor = new CANSparkMax(Constants.ShooterConstants.fwRightMotorNum, MotorType.kBrushless);
    indexerMotor = new CANSparkMax(Constants.ShooterConstants.indexerMotor, MotorType.kBrushless);
    pitchMotor = new CANSparkMax(Constants.ShooterConstants.pitchEncoder, MotorType.kBrushless);


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
    shooterAbsoluteEncoder = pitchMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    fwLeftPID = fwLeftMotor.getPIDController();
    fwRightPID = fwRightMotor.getPIDController();
    indexPID = indexerMotor.getPIDController();

    fwLeftPID.setP(Constants.ShooterConstants.kP_Shooter);
    fwLeftPID.setI(Constants.ShooterConstants.kI_Shooter);
    fwLeftPID.setD(Constants.ShooterConstants.kD_Shooter);

    fwRightPID.setP(Constants.ShooterConstants.kP_Shooter);
    fwRightPID.setI(Constants.ShooterConstants.kI_Shooter);
    fwRightPID.setD(Constants.ShooterConstants.kD_Shooter);

    indexPID.setP(Constants.ShooterConstants.kP_Shooter);
    indexPID.setI(Constants.ShooterConstants.kP_Shooter);
    indexPID.setD(Constants.ShooterConstants.kP_Shooter);

    shooterIndexer = new DigitalOutput(Constants.ShooterConstants.indexSensor);
}


//TODO need to write basic movement commands and fuctions


public Command upSpeedCommand() {
     return runOnce(() -> {
        fwLeftMotor.set(fwLeftMotor.get()+1);
        fwRightMotor.set(fwRightMotor.get()+1);
     });
}

public Command lowerSpeedCommand() {
    return runOnce(() -> {
        fwLeftMotor.set(fwLeftMotor.get()-1);
        fwRightMotor.set(fwRightMotor.get()-1);
    });
}


@Override
public void periodic() {
    SmartDashboard.putNumber("Speed of Shooter", fwLeftEncoder.getVelocity());
}

}
