package frc.robot.subsystems;

import org.ejml.ops.ComplexMath_F32;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    
    
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




    public IntakeSubsystem(){
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

    }

    public Command runIntakeCommand(){
        return runOnce(() -> {
            intLeftMotor.set(5);
            intRightMotor.set(5);
            handoffMotor.set(5);
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

    

}
