package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.*;



public class SwerveModule extends SubsystemBase{
    
  //initalize all variables
    public static CANSparkMax steerMotor;
    public CANSparkMax driveMotor;
    public final SparkPIDController turningPidController;


    private static final double RAMP_RATE = 0.85;//1.5;
    public RelativeEncoder driveMotorEncoder;
    public RelativeEncoder steerMotorEncoder;
    public CANcoder absoluteEncoder;
    private boolean absoluteEncoderReversed;
    public final SparkPIDController drivePidController;



  //New Swerve Module start
  public SwerveModule(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer, int absoluteEncoderId,
  double absoluteEncoderOffsetRad, Boolean absoluteEncoderReversed) {

    //Steer + Drive Motor Config
    driveMotor = new CANSparkMax(driveNum, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    driveMotor.restoreFactoryDefaults();
    driveMotor.setInverted(invertDrive);
    driveMotor.setOpenLoopRampRate(RAMP_RATE);
    driveMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
     
    steerMotor = new CANSparkMax(steerNum, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    steerMotor.restoreFactoryDefaults();
    steerMotor.setInverted(invertSteer);
    steerMotor.setOpenLoopRampRate(RAMP_RATE);
    steerMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);


    //Steer + Driving PID Controllers
    turningPidController = steerMotor.getPIDController();
    turningPidController.setP(Constants.ModuleConstants.kPTurning);
    turningPidController.setI(Constants.ModuleConstants.kITurning);
    turningPidController.setD(Constants.ModuleConstants.kITurning);
    turningPidController.setPositionPIDWrappingEnabled(true);
    turningPidController.setPositionPIDWrappingMaxInput(1080); 
    turningPidController.setPositionPIDWrappingMinInput(720);
    
    drivePidController = driveMotor.getPIDController();
    drivePidController.setP(0.95);
    
    //Absolute Encoder
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new CANcoder(absoluteEncoderId);
    
    
    //Steer + Drive Motor Encoder
    driveMotorEncoder = driveMotor.getEncoder();
    driveMotorEncoder.setPositionConversionFactor( 1/1);//23.58
    driveMotorEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    steerMotorEncoder = steerMotor.getEncoder();
    steerMotorEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningConversionFactor2Deg);
    steerMotorEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kTurningEncoderRPM2DegPerSec);

 //reset encoders after init phase
    resetDrive();
    System.out.println("reset encoders");
  }

public static SparkPIDController getPIDController() {
  return steerMotor.getPIDController();
}
public void resetDrive() {
  driveMotorEncoder.setPosition(0);
  steerMotorEncoder.setPosition(0);

}
//stop method that stops the motors when the stick/s are within the deadzone < 0.01
public void stop() {
  driveMotor.set(0);
  steerMotor.set(0);
}

  //configure our Absolute Encoder for the MK4 drive system
  CANcoderConfiguration config = new CANcoderConfiguration();
  public double getAbsoluteEncoderDeg(double AEOffset) {
//TODO See which one of these works
    double angle = absoluteEncoder.getPosition().getValueAsDouble();
    angle *= 360;
    return angle  * (absoluteEncoderReversed ? -1 : 1) - AEOffset;
  // return angle  * (absoluteEncoderReversed ? -1 : 1);
  }
  
  //Motor calls
  public double getDrivePosition() {
    return driveMotorEncoder.getPosition();}
  public double getDriveVelocity() {
    return driveMotorEncoder.getVelocity();}
  public double getSteerPosition() {
     return Math.abs(steerMotorEncoder.getPosition() % 360);}
  public double getSteerVelocity() {
    return steerMotorEncoder.getVelocity();}
  public double getPositionMeters() {
    return driveMotorEncoder.getPosition();}
  
 
  
//Creating the current state of the modules. A drive velo and an angle are needed. We use an off set of -90 for the angle
public SwerveModuleState gState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(steerMotorEncoder.getPosition()));
  }
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveMotorEncoder.getPosition(), Rotation2d.fromDegrees(steerMotorEncoder.getPosition()));
  }

//This is our setDesiredState alg. Takes the current state and the desired state shown by the controller and points the wheels to that 
//location
public void setDesiredState(SwerveModuleState state) {
if (Math.abs(state.speedMetersPerSecond) < 0.01) {stop();return;}
// state = optimizer2(state, gState().angle.getDegrees());
state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(gState().angle.getDegrees()));
driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
turningPidController.setReference(state.angle.getDegrees(), com.revrobotics.CANSparkBase.ControlType.kPosition);
}

public void wheelFaceForward(double AEOffset) {
  steerMotorEncoder.setPosition(getAbsoluteEncoderDeg(AEOffset));
  try{Thread.sleep(10);
    turningPidController.setReference(0, com.revrobotics.CANSparkBase.ControlType.kPosition);
  }catch (Exception e) {}}





//TODO see if the actual one works fine and then I wont need this code


// public SwerveModuleState optimizer2(SwerveModuleState desiredState, double currentAngle) {
//   var delta = desiredState.angle.minus(Rotation2d.fromDegrees(currentAngle));

//   if (Math.abs(delta.getDegrees()) > 90) {
//     return new SwerveModuleState(
//       -desiredState.speedMetersPerSecond,
//       (Rotation2d.fromDegrees(180).minus(desiredState.angle)).times(-1)
//     );
//   }
  
//   return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
//   }

}