package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    //laterator
    // public final CANSparkMax latLeftMotor;
    // public final CANSparkMax latRightMotor;

    // public final SparkPIDController latLeftPID;
    // public final SparkPIDController latRightPID;

    // public final RelativeEncoder latLeftEncoder;
    // public final RelativeEncoder latRightEncoder;
    // public final AbsoluteEncoder latAbsoluteEncoder;

    //intake
    public final CANSparkMax intLeftMotor;
    public final CANSparkMax intRightMotor;

    public final SparkPIDController intLeftPID;
    public final SparkPIDController intRightPID;

    public final RelativeEncoder intLeftEncoder;
    public final RelativeEncoder intRightEncoder;


    public final DoubleSolenoid intakeLift = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, Constants.intakeConstants.liftNum, Constants.intakeConstants.dropNum);
    public final Compressor intakeCompressor = new Compressor(1, PneumaticsModuleType.CTREPCM);

    public boolean isRetracted = true;
    //intakeIndexer
    public final DigitalInput intakeIndexer;

    public IntakeSubsystem(){
       

        //intake instatiation

        intLeftMotor = new CANSparkMax(Constants.intakeConstants.intLeftEncoder, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        intRightMotor = new CANSparkMax(Constants.intakeConstants.intRightEncoder, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

        intLeftMotor.restoreFactoryDefaults();
        intRightMotor.restoreFactoryDefaults();

        intLeftMotor.setInverted(Constants.intakeConstants.leftInverted);
        intRightMotor.setInverted(Constants.intakeConstants.rightInverted);

        intLeftMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
        intRightMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);

        intLeftMotor.setOpenLoopRampRate(Constants.intakeConstants.ramp_rate);
        intRightMotor.setOpenLoopRampRate(Constants.intakeConstants.ramp_rate);

        intLeftEncoder = intLeftMotor.getEncoder();
        intRightEncoder = intRightMotor.getEncoder();

        intLeftPID = intLeftMotor.getPIDController();
        intRightPID = intRightMotor.getPIDController();

        intLeftPID.setP(Constants.intakeConstants.kP_Intake);
        intLeftPID.setI(Constants.intakeConstants.kI_Intake);
        intLeftPID.setD(Constants.intakeConstants.kD_Intake);

        intRightPID.setP(Constants.intakeConstants.kP_Intake);
        intRightPID.setI(Constants.intakeConstants.kI_Intake);
        intRightPID.setD(Constants.intakeConstants.kD_Intake);

        intakeIndexer = new DigitalInput(Constants.intakeConstants.indexingSensor);

        zeroAll();
        intakeLift.set(Value.kForward);
    }
    
    public void zeroAll(){
        intLeftEncoder.setPosition(0);
        intRightEncoder.setPosition(0);
    }
    //Commands

    public double setSpeed = 0;
    public Command runIntakeCommand(){
        return runOnce(() -> {
            setSpeed = 1;
            intLeftMotor.set(setSpeed);
            intRightMotor.set(setSpeed);
    });}
    public Command reverseIntakecommand(){
        return runOnce(() -> {
            setSpeed = -1;
            intLeftMotor.set(setSpeed);
            intRightMotor.set(setSpeed);
        });
    }
    public Command stopIntakeCommand(){
        return runOnce(() -> {
            intLeftMotor.set(0);
            intRightMotor.set(0);
    });}

    //Pneumatics
    public Command liftIntakeCommand(){
        return runOnce(() -> {
            intakeLift.set(Value.kForward);
            isRetracted = true;
    });}
    public Command dropIntakeCommand(){
        return runOnce(() -> {
            intakeLift.set(Value.kReverse);
            isRetracted = false;
    });}

    public SequentialCommandGroup deployCommand(){
        return dropIntakeCommand().andThen(runIntakeCommand());
    }
    public SequentialCommandGroup retractCommand(){
        return liftIntakeCommand().andThen(stopIntakeCommand());
    }


    @Override
    public void periodic(){

      
        
    }


}
