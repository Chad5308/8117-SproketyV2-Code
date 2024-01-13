package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase{
    
public SwerveSubsystem s_swerve;
public NetworkTable networkTables;
public IntegerSubscriber pipeline;
public IntegerPublisher pipelinePublisher;
public double targetHeight = 36.5;
public double cameraHeight = 39;
public double xPos, yPos, hasTargets, targetNum, xSpeed, ySpeed, turningSpeed;
public double correctionX, correctionY, correctionT;
public double distanceX, distanceY;
public double[] localizedPose;
public ProfiledPIDController thetaPIDController;
public ProfiledPIDController linearPIDController;






    public LimelightSubsystem(SwerveSubsystem s_swerve){
        this.s_swerve = s_swerve;

//NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);



        networkTables = NetworkTableInstance.getDefault().getTable("limelight");

        pipeline = networkTables.getIntegerTopic("limelight.getpipe").subscribe(0);
        pipelinePublisher = networkTables.getIntegerTopic("limelight.getpipeline").publish();
        thetaPIDController = new ProfiledPIDController(Constants.limelightConstants.kP, Constants.limelightConstants.kI, Constants.limelightConstants.kD, Constants.AutoConstants.kThetaControllerConstraints);
        linearPIDController = new ProfiledPIDController(Constants.limelightConstants.kP, Constants.limelightConstants.kI, Constants.limelightConstants.kD, Constants.AutoConstants.kLinearConstraints);
    }

    public Optional<Pose2d> getPoseFromAprilTags() {
        double[] botpose = localizedPose;
        if(botpose.length < 7 || targetNum == -1) return Optional.empty();
        return Optional.of(new Pose2d(botpose[0], botpose[1], new Rotation2d(botpose[5])));
    }

    public void setThetaPID(){
        thetaPIDController.setGoal(0);
        thetaPIDController.setTolerance(Math.toRadians(5));
        }
    public void setLinearPID(){
        linearPIDController.setGoal(1.25);
        linearPIDController.setTolerance(1.5);
    }
    

@Override
public void periodic(){
    if (s_swerve.allianceCheck() == true) {
            localizedPose = networkTables.getEntry("botpose_wpired").getDoubleArray(new double[] {});
    } else {
            localizedPose = networkTables.getEntry("botpose_wpiblue").getDoubleArray(new double[] {});
        }
    
 // double currentTargetX = Math.toRadians(-xPos.get());
xPos = Math.toRadians(networkTables.getEntry("tx").getDouble(0));
yPos = Math.toRadians(networkTables.getEntry("ty").getDouble(0));
hasTargets = networkTables.getEntry("tv").getDouble(0);
targetNum = networkTables.getEntry("tid").getDouble(0);
distanceX = Math.abs(((targetHeight-cameraHeight) / Math.tan((yPos)) * 0.0235));
distanceY = distanceX * Math.tan(xPos);

 
 

//  currentTargetY = Math.toRadians(yPos);
correctionX = linearPIDController.calculate(distanceX);
correctionY = linearPIDController.calculate(distanceY);
correctionT = thetaPIDController.calculate(xPos);

// if(!pidController.atSetpoint()){
//     turningSpeed = pidController.getSetpoint().velocity + correction;
// }else {
//     turningSpeed = 0;
// }




if(!(thetaPIDController.atSetpoint() && linearPIDController.atSetpoint())){
    xSpeed = linearPIDController.getSetpoint().velocity + correctionX;
    ySpeed = linearPIDController.getSetpoint().velocity + correctionY;
    turningSpeed = thetaPIDController.getSetpoint().velocity + correctionT;
}else {
    turningSpeed = 0;
}

SmartDashboard.putNumber("Distance X", distanceX);
SmartDashboard.putNumber("Distance Y", distanceY);
SmartDashboard.putNumber("Correction Theta", turningSpeed);
SmartDashboard.putNumber("Correction X", xSpeed);
SmartDashboard.putNumber("Correction Y", ySpeed);
SmartDashboard.putNumber("TX Value", xPos);
SmartDashboard.putNumber("TY Value", yPos);


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

