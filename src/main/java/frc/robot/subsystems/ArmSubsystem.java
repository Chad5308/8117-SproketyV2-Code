package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
    //laterator
    public final CANSparkMax latLeftMotor;
    public final CANSparkMax latRightMotor;

    public final SparkPIDController latLeftPID;
    public final SparkPIDController latRightPID;

    public final RelativeEncoder latLeftEncoder;
    public final RelativeEncoder latRightEncoder;
    public final AbsoluteEncoder latAbsoluteEncoder;

    //intake
    public final CANSparkMax intLeftMotor;
    public final CANSparkMax intRightMotor;
    public final CANSparkMax handoffMotor;

    public final SparkPIDController intLeftPID;
    public final SparkPIDController intRightPID;
    public final SparkPIDController handoffPID;

    public final RelativeEncoder intLeftEncoder;
    public final RelativeEncoder intRightEncoder;
    public final RelativeEncoder handoffEncoder;


    public final DoubleSolenoid intakeLift = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, Constants.intakeConstants.liftNum, Constants.intakeConstants.dropNum);
    public final Compressor intakeCompressor = new Compressor(1, PneumaticsModuleType.CTREPCM);


    //intakeIndexer
    public final DigitalOutput intakeIndexer;

    public ArmSubsystem(){
        //Laterator motor instatiation
        latLeftMotor = new CANSparkMax(Constants.LateratorConstants.leftMotor, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        latRightMotor = new CANSparkMax(Constants.LateratorConstants.rightMotor, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

        latLeftMotor.restoreFactoryDefaults();
        latRightMotor.restoreFactoryDefaults();

        latLeftMotor.setInverted(Constants.LateratorConstants.leftInverted);
        latRightMotor.setInverted(Constants.LateratorConstants.rightInverted);

        latLeftMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
        latRightMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);

        latLeftMotor.setOpenLoopRampRate(Constants.LateratorConstants.ramp_rate);
        latRightMotor.setOpenLoopRampRate(Constants.LateratorConstants.ramp_rate);

        latLeftEncoder = latLeftMotor.getEncoder();
        latRightEncoder = latRightMotor.getEncoder();
        latAbsoluteEncoder = latLeftMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        latLeftPID = latLeftMotor.getPIDController();
        latRightPID = latRightMotor.getPIDController();

        latLeftPID.setP(Constants.LateratorConstants.kP_Laterator);
        latLeftPID.setI(Constants.LateratorConstants.kI_Laterator);
        latLeftPID.setD(Constants.LateratorConstants.kD_Laterator);

        latRightPID.setP(Constants.LateratorConstants.kP_Laterator);
        latRightPID.setI(Constants.LateratorConstants.kI_Laterator);
        latRightPID.setD(Constants.LateratorConstants.kD_Laterator);

        //intake instatiation

        intLeftMotor = new CANSparkMax(Constants.intakeConstants.intLeftEncoder, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        intRightMotor = new CANSparkMax(Constants.intakeConstants.intRightEncoder, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        handoffMotor = new CANSparkMax(Constants.intakeConstants.handoffEncoder, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

        intLeftMotor.restoreFactoryDefaults();
        intRightMotor.restoreFactoryDefaults();
        handoffMotor.restoreFactoryDefaults();

        intLeftMotor.setInverted(Constants.intakeConstants.leftInverted);
        intRightMotor.setInverted(Constants.intakeConstants.rightInverted);
        handoffMotor.setInverted(Constants.intakeConstants.handoffInverted);

        intLeftMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
        intRightMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
        handoffMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);

        intLeftMotor.setOpenLoopRampRate(Constants.intakeConstants.ramp_rate);
        intRightMotor.setOpenLoopRampRate(Constants.intakeConstants.ramp_rate);
        handoffMotor.setOpenLoopRampRate(Constants.intakeConstants.ramp_rate);

        intLeftEncoder = intLeftMotor.getEncoder();
        intRightEncoder = intRightMotor.getEncoder();
        handoffEncoder = handoffMotor.getEncoder();

        intLeftPID = intLeftMotor.getPIDController();
        intRightPID = intRightMotor.getPIDController();
        handoffPID = handoffMotor.getPIDController();

        intLeftPID.setP(Constants.intakeConstants.kP_Intake);
        intLeftPID.setI(Constants.intakeConstants.kI_Intake);
        intLeftPID.setD(Constants.intakeConstants.kD_Intake);

        intRightPID.setP(Constants.intakeConstants.kP_Intake);
        intRightPID.setI(Constants.intakeConstants.kI_Intake);
        intRightPID.setD(Constants.intakeConstants.kD_Intake);

        handoffPID.setP(Constants.intakeConstants.kP_Handoff);
        handoffPID.setI(Constants.intakeConstants.kI_Handoff);
        handoffPID.setD(Constants.intakeConstants.kD_Handoff);


        intakeIndexer = new DigitalOutput(Constants.intakeConstants.indexingSensor);


        zeroAll();
    }

    public void zeroAll(){
        latLeftEncoder.setPosition(latAbsoluteEncoder.getPosition());
        latRightEncoder.setPosition(latAbsoluteEncoder.getPosition());

        intLeftEncoder.setPosition(0);
        intRightEncoder.setPosition(0);
        handoffEncoder.setPosition(0);
    }
    //Commands

    public Command runIntakeCommand(){
        return runOnce(() -> {
            intLeftMotor.set(5);
            intRightMotor.set(5);
            handoffMotor.set(5);
        });
    }

    public Command stopIntakeCommand(){
        return runOnce(() -> {
            intLeftMotor.set(0);
            intRightMotor.set(0);
        });
    }
    
    public Command liftIntakeCommand(){
        return runOnce(() -> {
            intakeLift.set(Value.kForward);
        });
    }

    public Command dropIntakeCommand(){
        return runOnce(() -> {
            intakeLift.set(Value.kReverse);
        });
    }




//laterator Commands


//TODO work out these values
    public Command fullExtensionCommand(){
        return runOnce(() -> {
            latLeftPID.setReference(1, ControlType.kPosition);
            latRightPID.setReference(1, ControlType.kPosition);
        });
    }

    public Command indexPositionCommand(){
        return runOnce(() -> {
            latLeftPID.setReference(1, ControlType.kPosition);
            latRightPID.setReference(1, ControlType.kPosition);
        });
    }

    public Command ampPosition(){
        return runOnce(() -> {
            latLeftPID.setReference(1, ControlType.kPosition);
            latRightPID.setReference(1, ControlType.kPosition);
        });
    }

    



    @Override
    public void periodic(){


        if(intakeIndexer.get()){
            runOnce((Runnable) stopIntakeCommand());
            runOnce((Runnable) indexPositionCommand());
            
            //TODO ended here, need to finish intakeIndexer algorithm
        }
    }


}
