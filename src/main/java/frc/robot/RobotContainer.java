// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  // private CommandJoystick driveStick = new CommandJoystick(0);
  public static Robot robot = new Robot();
  public SwerveSubsystem s_Swerve = new SwerveSubsystem(robot);
  public DriveCommand d_Command = new DriveCommand(s_Swerve, opController); 
  public PneumaticsSubsystem p_sub = new PneumaticsSubsystem();
  public IntakeSubsystem int_sub = new IntakeSubsystem();
  private SendableChooser<Command> autoChooser;
  public LimelightSubsystem LL_sub = new LimelightSubsystem(s_Swerve);
  public LimelightCommand LL_com = new LimelightCommand(s_Swerve, LL_sub);



//TODO see if this works now


public RobotContainer() {
  // s_Swerve.setDefaultCommand(new DriveCommand(s_Swerve, opController));
  s_Swerve.setDefaultCommand(new LimelightCommand(s_Swerve, LL_sub));
  configureBindings();
  
  AutoBuilder.configureHolonomic(
      s_Swerve::getPose, 
      s_Swerve::resetOdometry, 
      d_Command::getSpeeds, 
      s_Swerve::setModuleStates, 
      new HolonomicPathFollowerConfig(new PIDConstants(0.025, 0, 0), new PIDConstants(0.025, 0, 0), 
      Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, 
      Constants.ModuleConstants.moduleRadius, 
      new ReplanningConfig(true, true)), 
      s_Swerve::allianceCheck,
      s_Swerve);
      

      
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);



    NamedCommands.registerCommand("Run Intake", int_sub.runIntakeCommand());
    
  }

  public Command testAuto(){
        return new PathPlannerAuto("Test Auto");
    }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  
  private void configureBindings() {
    opController.povRight().toggleOnTrue(s_Swerve.zeroHeadingCommand());
    opController.povLeft().toggleOnTrue(s_Swerve.fieldOrientedToggle());
    opController.button(7).toggleOnTrue(s_Swerve.resetWheels());

  }
}
