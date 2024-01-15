package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase{
    
public SwerveSubsystem s_swerve;
public NetworkTable networkTables;
public IntegerSubscriber pipeline;
public IntegerPublisher pipelinePublisher;
public double targetHeight = 36.5;
public double cameraHeight = 39;
public double xAng, yAng, hasTargets, targetNum, xSpeed, ySpeed, turningSpeed;
public double correctionX, correctionY, correctionT;
public double distanceX, distanceY;
public double[] localizedPose;
public ProfiledPIDController thetaPIDController;
public ProfiledPIDController linearPIDController;
public boolean autoAlign = false;






    public LimelightSubsystem(SwerveSubsystem s_swerve){
        this.s_swerve = s_swerve;

//NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);



        networkTables = NetworkTableInstance.getDefault().getTable("limelight");

        pipeline = networkTables.getIntegerTopic("limelight.getpipe").subscribe(0);
        pipelinePublisher = networkTables.getIntegerTopic("limelight.getpipeline").publish();
        thetaPIDController = new ProfiledPIDController(Constants.limelightConstants.thetakP, Constants.limelightConstants.thetakI, Constants.limelightConstants.thetakD, Constants.AutoConstants.kThetaControllerConstraints);
        linearPIDController = new ProfiledPIDController(Constants.limelightConstants.linearkP, Constants.limelightConstants.linearkI, Constants.limelightConstants.linearkD, Constants.AutoConstants.kLinearConstraints);
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
        linearPIDController.setGoal(16); //inches
        linearPIDController.setTolerance(20); //inches
    }
    public Command autoAlignCommand(){
        return runOnce(() -> {
            autoAlign = true;
        });
    }
    

@Override
public void periodic(){
    if (s_swerve.allianceCheck() == true) {
            localizedPose = networkTables.getEntry("botpose_wpired").getDoubleArray(new double[] {});
    } else {
            localizedPose = networkTables.getEntry("botpose_wpiblue").getDoubleArray(new double[] {});
        }
        
    
 // double currentTargetX = Math.toRadians(-xPos.get());
xAng = Math.toRadians(networkTables.getEntry("tx").getDouble(0));
yAng = Math.toRadians(networkTables.getEntry("ty").getDouble(0));
hasTargets = networkTables.getEntry("tv").getDouble(0);
targetNum = networkTables.getEntry("tid").getDouble(0);
distanceX = (targetHeight-cameraHeight) / (Math.tan(yAng));//inches
distanceY = distanceX * Math.tan(xAng);//inches

 
//  currentTargetY = Math.toRadians(yPos);
correctionX = linearPIDController.calculate(distanceX);//inches
correctionY = linearPIDController.calculate(distanceY);//inches
correctionT = thetaPIDController.calculate(xAng);//radians

// if(!thetaPIDController.atSetpoint()){
//     turningSpeed = thetaPIDController.getSetpoint().velocity + correctionT;
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
SmartDashboard.putNumber("TX Value", xAng);
SmartDashboard.putNumber("TY Value", yAng);

}

}
