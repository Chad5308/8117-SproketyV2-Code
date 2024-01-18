package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants;

public class ShooterSubsystem {
    
public final CANSparkMax fwLeftMotor;
public final CANSparkMax fwRightMotor;

public final RelativeEncoder fwLeftEncoder;
public final RelativeEncoder fwRightEncoder;

public final SparkPIDController fwLeftPID;
public final SparkPIDController fwRightPID;



public ShooterSubsystem(){
    fwLeftMotor = new CANSparkMax(Constants.ShooterConstants.fwLeftMotorNum, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    fwRightMotor = new CANSparkMax(Constants.ShooterConstants.fwRightMotorNum, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    fwLeftMotor.restoreFactoryDefaults();
    fwRightMotor.restoreFactoryDefaults();

    fwLeftMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
    fwRightMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);

    fwLeftMotor.setOpenLoopRampRate(Constants.ShooterConstants.ramp_rate);
    fwRightMotor.setOpenLoopRampRate(Constants.ShooterConstants.ramp_rate);

    fwLeftEncoder = fwLeftMotor.getEncoder();
    fwRightEncoder = fwRightMotor.getEncoder();

    fwLeftPID = fwLeftMotor.getPIDController();
    fwRightPID = fwRightMotor.getPIDController();

    fwLeftPID.setP(Constants.ShooterConstants.kP_Shooter);
    fwLeftPID.setI(Constants.ShooterConstants.kI_Shooter);
    fwLeftPID.setD(Constants.ShooterConstants.kD_Shooter);

    fwRightPID.setP(Constants.ShooterConstants.kP_Shooter);
    fwRightPID.setI(Constants.ShooterConstants.kI_Shooter);
    fwRightPID.setD(Constants.ShooterConstants.kD_Shooter);
}

}
