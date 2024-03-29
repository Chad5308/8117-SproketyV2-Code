package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    DigitalInput intakeLimitSwitch;
    public boolean transferReady;

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

        intakeLimitSwitch = new DigitalInput(0);
        zeroAll();
        intakeLift.set(Value.kForward);
    }
    
    public void zeroAll(){
        intLeftEncoder.setPosition(0);
        intRightEncoder.setPosition(0);
    }
    //Commands
    
    public void setSpeed(double speed){
        intLeftMotor.set(speed);
        intRightMotor.set(speed);        
    }

    public double getSpeed(){
       return (intLeftMotor.get() + intRightMotor.get())/2;
    }

    public Command runIntakeCommand(){
        return runOnce(() -> {
            setSpeed(1);
    });}
    public Command reverseIntakecommand(){
        return runOnce(() -> {
            setSpeed(-1);
        });
    }
    public Command stopIntakeCommand(){
        return runOnce(() -> {
            setSpeed(0);
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
        // SmartDashboard.putBoolean("Intake Switch", intakeLimitSwitch.get());
        // transferReady = (!isRetracted && getSpeed() > 0 && intakeLimitSwitch.get() == true);
        // SmartDashboard.putBoolean("Transfer Boolean", transferReady);

      
        
    }


}
