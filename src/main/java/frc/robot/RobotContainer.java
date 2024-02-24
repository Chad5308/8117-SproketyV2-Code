// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Drivebase.LimelightSubsystem;
import frc.robot.subsystems.Drivebase.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;



/**
 * This class is where the bulk
 *  of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController opController = new CommandXboxController(OIConstants.kOPControllerPort);
  private final CommandXboxController shootController = new CommandXboxController(0);
  // private CommandJoystick driveStick = new CommandJoystick(0);
  public static Robot robot = new Robot();
  public SwerveSubsystem s_Swerve = new SwerveSubsystem(robot);
  private SendableChooser<Command> autoChooser;
  public LimelightSubsystem LL_sub = new LimelightSubsystem(s_Swerve);
  public DriveCommand d_Command = new DriveCommand(s_Swerve, LL_sub, opController);
  public ShooterSubsystem shooter_sub = new ShooterSubsystem();
  public ArmSubsystem arm_sub = new ArmSubsystem();



public RobotContainer() {
  s_Swerve.setDefaultCommand(new DriveCommand(s_Swerve, LL_sub, opController));
  configureBindings();
  
  AutoBuilder.configureHolonomic(
      s_Swerve::getPose, 
      s_Swerve::resetOdometry, 
      d_Command::getSpeeds, 
      s_Swerve::setModuleStates, 
      new HolonomicPathFollowerConfig(new PIDConstants(Constants.AutoConstants.kPTranslation, Constants.AutoConstants.kITranslation, Constants.AutoConstants.kDTranslation), new PIDConstants(Constants.AutoConstants.kITheta, Constants.AutoConstants.kITheta, Constants.AutoConstants.kDTheta), 
      Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, 
      Constants.ModuleConstants.moduleRadius, 
      new ReplanningConfig(true, true)), 
      s_Swerve::allianceCheck,
      s_Swerve);
      

      
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);    


    NamedCommands.registerCommand("FaceForward Wheels", s_Swerve.faceForwardCommand());
    NamedCommands.registerCommand("Deploy", arm_sub.deployCommand());
    NamedCommands.registerCommand("Shoot", shooter_sub.closeSpeakerCommand().andThen(Commands.waitSeconds(2)).andThen(shooter_sub.shootCommand().andThen(Commands.waitSeconds(2)).andThen(shooter_sub.stopFWCommand())));
  }

  public void configureAutos(){
    autoChooser.addOption("2 Piece Auto", TwoPieceAuto());
    autoChooser.addOption("Leave Auto", leaveAuto());
    autoChooser.addOption("Score + Leave", scoreLeaveAuto());
  }

  public Command TwoPieceAuto(){
        return new PathPlannerAuto("2 Piece Auto");
    }
  public Command leaveAuto(){
    return new PathPlannerAuto("Leave Auto");
  }
  public Command scoreLeaveAuto(){
    return new PathPlannerAuto("Score + Leave Auto");
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  
  private void configureBindings() {
    //Drive Controls
    opController.povRight().toggleOnTrue(s_Swerve.zeroHeadingCommand());
    opController.povLeft().toggleOnTrue(s_Swerve.fieldOrientedToggle());
    opController.button(7).onTrue(s_Swerve.resetWheels()); //window looking button
    opController.axisGreaterThan(3, 0.25).whileTrue(arm_sub.deployCommand());
    opController.axisGreaterThan(3, 0.25).whileFalse(arm_sub.stopIntakeCommand());

    opController.a().onTrue(arm_sub.liftIntakeCommand());
    opController.b().onTrue(arm_sub.dropIntakeCommand());
    // opController.povUp().onTrue(LL_sub.autoAlignCommand());

    //shooter Controls
    shootController.x().whileTrue(shooter_sub.rotateOutCommand());
    shootController.b().whileTrue(shooter_sub.rotateInCommand());
    shootController.x().whileFalse(shooter_sub.pitchStopCommand());
    shootController.b().whileFalse(shooter_sub.pitchStopCommand());
    shootController.axisGreaterThan(3, 0.25).whileTrue(shooter_sub.shootCommand());
    shootController.axisGreaterThan(3, 0.25).whileFalse(shooter_sub.stopFWCommand());

    // opController.y().onTrue(shooter_sub.upSpeedCommand());
    // opController.a().onTrue(shooter_sub.lowerSpeedCommand());


    // //Laterator Controlls

    // opController.axisGreaterThan(3, 0.5).onTrue(arm_sub.liftLaterator());
    // opController.axisGreaterThan(3, 0.5).onFalse(arm_sub.stopLaterator());
    // opController.axisGreaterThan(4, 0.5).onTrue(arm_sub.lowerLaterator());
    // opController.axisGreaterThan(4, 0.5).onFalse(arm_sub.stopLaterator());
    
    //Auto fire Controls
    // opController.axisGreaterThan(3, 0.5).onTrue(shooter_sub.closeSpeakerCommand());


  }
}
