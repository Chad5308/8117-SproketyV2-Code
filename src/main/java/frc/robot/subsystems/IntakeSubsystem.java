package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    
    
public final CANSparkMax intakePair1Motor;
public final CANSparkMax intakePair2Motor;
public final CANSparkMax handoffMotor;

public final SparkPIDController intakePair1PID;
public final SparkPIDController intakePair2PID;
public final SparkPIDController handoffPID;

public final RelativeEncoder intakePair1Encoder;
public final RelativeEncoder intakePair2Encoder;
public final RelativeEncoder handoffEncoder;




    public IntakeSubsystem(){
        intakePair1Motor = new CANSparkMax(Constants.intakeConstants.intakePair1Encoder, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        intakePair2Motor = new CANSparkMax(Constants.intakeConstants.intakePair2Encoder, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        handoffMotor = new CANSparkMax(Constants.intakeConstants.handoffEncoder, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

        intakePair1Motor.restoreFactoryDefaults();
        intakePair2Motor.restoreFactoryDefaults();
        handoffMotor.restoreFactoryDefaults();

        intakePair1Motor.setInverted(Constants.intakeConstants.pair1Inverted);
        intakePair2Motor.setInverted(Constants.intakeConstants.pair2Inverted);
        handoffMotor.setInverted(Constants.intakeConstants.handoffInverted);

        intakePair1Motor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
        intakePair2Motor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
        handoffMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);

        intakePair1Motor.setOpenLoopRampRate(Constants.intakeConstants.ramp_rate);
        intakePair2Motor.setOpenLoopRampRate(Constants.intakeConstants.ramp_rate);
        handoffMotor.setOpenLoopRampRate(Constants.intakeConstants.ramp_rate);

        intakePair1Encoder = intakePair1Motor.getEncoder();
        intakePair2Encoder = intakePair2Motor.getEncoder();
        handoffEncoder = handoffMotor.getEncoder();

        intakePair1PID = intakePair1Motor.getPIDController();
        intakePair2PID = intakePair2Motor.getPIDController();
        handoffPID = handoffMotor.getPIDController();

        intakePair1PID.setP(Constants.intakeConstants.kP_Intake);
        intakePair1PID.setI(Constants.intakeConstants.kI_Intake);
        intakePair1PID.setD(Constants.intakeConstants.kD_Intake);

        intakePair2PID.setP(Constants.intakeConstants.kP_Intake);
        intakePair2PID.setI(Constants.intakeConstants.kI_Intake);
        intakePair2PID.setD(Constants.intakeConstants.kD_Intake);

        handoffPID.setP(Constants.intakeConstants.kP_Handoff);
        handoffPID.setI(Constants.intakeConstants.kI_Handoff);
        handoffPID.setD(Constants.intakeConstants.kD_Handoff);



    }

}
