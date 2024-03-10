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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;



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
  private SendableChooser<Command> autoChooser;
  public ShooterSubsystem shooter_sub = new ShooterSubsystem();
  public ArmSubsystem arm_sub = new ArmSubsystem();
  public SwerveSubsystem s_Swerve = new SwerveSubsystem(robot, shooter_sub, arm_sub);
  public LimelightSubsystem LL_sub = new LimelightSubsystem(s_Swerve);
  public DriveCommand d_Command = new DriveCommand(s_Swerve, LL_sub, opController);



public RobotContainer() {
  s_Swerve.setDefaultCommand(new DriveCommand(s_Swerve, LL_sub, opController));
  configureBindings();


      
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);    


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
    return new PathPlannerAuto("leave Auto");
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
    opController.axisGreaterThan(3, 0.25).whileTrue(arm_sub.runIntakeCommand());
    opController.axisGreaterThan(3, 0.25).whileFalse(arm_sub.stopIntakeCommand());
    opController.axisGreaterThan(2, 0.25).whileTrue(arm_sub.dropIntakeCommand());
    opController.axisGreaterThan(2, 0.25).whileFalse(arm_sub.liftIntakeCommand());

    opController.a().onTrue(arm_sub.liftIntakeCommand());
    opController.b().onTrue(arm_sub.dropIntakeCommand());
    // opController.povUp().onTrue(LL_sub.autoAlignCommand());

    //shooter Controls
    shootController.x().whileTrue(shooter_sub.rotateOutCommand());
    shootController.b().whileTrue(shooter_sub.rotateInCommand());
    shootController.x().whileFalse(shooter_sub.pitchStopCommand());
    shootController.b().whileFalse(shooter_sub.pitchStopCommand());
    shootController.axisGreaterThan(3, 0.25).whileTrue(shooter_sub.rampUpCommand());
    shootController.rightBumper().whileTrue(shooter_sub.fireCommand());
    shootController.rightBumper().whileFalse(shooter_sub.stopFWCommand());
    shootController.axisGreaterThan(3, 0.25).whileFalse(shooter_sub.stopFWCommand());

    // shootController.povUp().onTrue(shooter_sub.testCommand());
    shootController.axisGreaterThan(2, 0.25).whileTrue(shooter_sub.indexShooterCommand());
    shootController.axisGreaterThan(2, 0.25).whileFalse(shooter_sub.stopFWCommand());
    shootController.povUp().whileTrue(arm_sub.runIntakeCommand());
    shootController.povUp().whileFalse(arm_sub.stopIntakeCommand());
    shootController.povDown().whileTrue(arm_sub.reverseIntakecommand());
    shootController.povDown().whileFalse(arm_sub.stopIntakeCommand());
    shootController.povLeft().whileTrue(shooter_sub.sourceIntakeCommand());
    shootController.povLeft().whileFalse(shooter_sub.stopFWCommand());
    shootController.povRight().whileTrue(shooter_sub.reverseIndexShooterCommand());
    shootController.povRight().whileFalse(shooter_sub.stopFWCommand());

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
