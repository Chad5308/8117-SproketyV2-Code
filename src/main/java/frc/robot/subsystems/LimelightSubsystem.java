package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase{
    

public DoubleSubscriber xPos;
public DoubleSubscriber yPos;
public IntegerSubscriber pipeline;
public IntegerPublisher pipelinePublisher;
public IntegerSubscriber hasTargets;
public SwerveSubsystem s_swerve;

public double xSpeed;
public double ySpeed;
public double turningSpeed;
public ProfiledPIDController pidController;




    public LimelightSubsystem(SwerveSubsystem s_swerve){
        this.s_swerve = s_swerve;

        NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
        hasTargets = networkTables.getIntegerTopic("Limelight/tv").subscribe(0);
        xPos = networkTables.getDoubleTopic("Limelight/tx").subscribe(0);
        yPos = networkTables.getDoubleTopic("Limelight/ty").subscribe(0);
        pipeline = networkTables.getIntegerTopic("Limelight.getpipe").subscribe(0);
        pipelinePublisher = networkTables.getIntegerTopic("Limelight.getpipeline").publish();
        pidController = new ProfiledPIDController(Constants.limelightConstants.kP, Constants.limelightConstants.kI, Constants.limelightConstants.kD, Constants.AutoConstants.kThetaControllerConstraints);
    }

    public void setPID(){
        pidController.setGoal(0);
        pidController.setTolerance(Math.toRadians(2));
        }

@Override
public void periodic(){
    double currentTargetX = Math.toRadians(-xPos.get());
    double currentTargetY = Math.toRadians(yPos.get());
    double correction = pidController.calculate(currentTargetX);
    
    if(!pidController.atSetpoint()){
        turningSpeed = pidController.getSetpoint().velocity + correction;
    }else {
        turningSpeed = 0;
    }
    
    SmartDashboard.putNumber("TX Value", currentTargetX);
    SmartDashboard.putNumber("TY Value", currentTargetY);



}

}

// NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
// NetworkTableEntry tx = table.getEntry("tx");
// NetworkTableEntry ty = table.getEntry("ty");
// NetworkTableEntry ta = table.getEntry("ta");

// //read values periodically
// double x = tx.getDouble(0.0);
// double y = ty.getDouble(0.0);
// double area = ta.getDouble(0.0);

