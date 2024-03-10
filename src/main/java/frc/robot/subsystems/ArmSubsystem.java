package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
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


    //intakeIndexer
    public final DigitalInput intakeIndexer;

    public ArmSubsystem(){
        //Laterator motor instatiation
        // latLeftMotor = new CANSparkMax(Constants.LateratorConstants.leftMotor, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        // latRightMotor = new CANSparkMax(Constants.LateratorConstants.rightMotor, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

        // latLeftMotor.restoreFactoryDefaults();
        // latRightMotor.restoreFactoryDefaults();

        // latLeftMotor.setInverted(Constants.LateratorConstants.leftInverted);
        // latRightMotor.setInverted(Constants.LateratorConstants.rightInverted);

        // latLeftMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
        // latRightMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);

        // latLeftMotor.setOpenLoopRampRate(Constants.LateratorConstants.ramp_rate);
        // latRightMotor.setOpenLoopRampRate(Constants.LateratorConstants.ramp_rate);

        // latLeftEncoder = latLeftMotor.getEncoder();
        // latRightEncoder = latRightMotor.getEncoder();
        // latAbsoluteEncoder = latLeftMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        // latLeftPID = latLeftMotor.getPIDController();
        // latRightPID = latRightMotor.getPIDController();

        // latLeftPID.setP(Constants.LateratorConstants.kP_Laterator);
        // latLeftPID.setI(Constants.LateratorConstants.kI_Laterator);
        // latLeftPID.setD(Constants.LateratorConstants.kD_Laterator);

        // latRightPID.setP(Constants.LateratorConstants.kP_Laterator);
        // latRightPID.setI(Constants.LateratorConstants.kI_Laterator);
        // latRightPID.setD(Constants.LateratorConstants.kD_Laterator);

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
        // latLeftEncoder.setPosition(latAbsoluteEncoder.getPosition());
        // latRightEncoder.setPosition(latAbsoluteEncoder.getPosition());

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
    public Command liftIntakeCommand(){
        return runOnce(() -> {
            intakeLift.set(Value.kForward);
    });}
    public Command dropIntakeCommand(){
        return runOnce(() -> {
            intakeLift.set(Value.kReverse);
    });}

    public SequentialCommandGroup deployCommand(){
        return dropIntakeCommand().andThen(runIntakeCommand());
    }
    // public SequentialCommandGroup retractCommand(){
    //     return liftIntakeCommand().andThen(stopIntakeCommand());
    // }




//laterator Commands

// public void setPosition(double position){
//     latLeftPID.setReference(position, ControlType.kPosition);
//     latRightPID.setReference(position, ControlType.kPosition);
// }


//TODO work out these values
    // public Command fullExtensionCommand(){
    //     return runOnce(() -> {
    //         setPosition(1);
    // });}
    // public Command indexPositionCommand(){
    //     return runOnce(() -> {
    //         setPosition(1);
    // });}
    // public Command ampPosition(){
    //     return runOnce(() -> {
    //         setPosition(1);
    // });}
    // public Command liftLaterator(){
    //     return runOnce(() -> {
    //         latLeftPID.setReference(1, ControlType.kVelocity);
    //         latRightPID.setReference(1, ControlType.kVelocity);
    // });}
    // public Command lowerLaterator(){
    //     return runOnce(() -> {
    //         latLeftPID.setReference(1, ControlType.kVelocity);
    //         latRightPID.setReference(1, ControlType.kVelocity);
    // });}
    // public Command stopLaterator(){
    //     return runOnce(() -> {
    //         latLeftPID.setReference(0, ControlType.kVelocity);
    //         latRightPID.setReference(0, ControlType.kVelocity);
    //     });
    // }
    



    @Override
    public void periodic(){


        if(intakeIndexer.get()){
            // retractCommand();          
        }


        
    }


}
