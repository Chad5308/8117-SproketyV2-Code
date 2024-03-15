// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShootingCommand;
import frc.robot.commands.TransferCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PitchSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Drivebase.LimelightSubsystem;
import frc.robot.subsystems.Drivebase.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final CommandXboxController shootController = new CommandXboxController(Constants.OIConstants.kShooterControllerPort);
  // private CommandJoystick driveStick = new CommandJoystick(0);
  public static Robot robot = new Robot();
  public ShooterSubsystem shooter_sub = new ShooterSubsystem();
  public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public PitchSubsystem pitchSubsystem = new PitchSubsystem();
  public ShootingCommand shootingCommand = new ShootingCommand(shooter_sub);
  public SwerveSubsystem s_Swerve = new SwerveSubsystem(robot, shooter_sub, intakeSubsystem);
  public LimelightSubsystem LL_sub = new LimelightSubsystem(s_Swerve);
  public DriveCommand d_Command = new DriveCommand(s_Swerve, LL_sub, opController);
  public TransferCommand transferCommand = new TransferCommand(shooter_sub, intakeSubsystem, pitchSubsystem);
  public AutoCommand autoCommand = new AutoCommand(shootingCommand, transferCommand, d_Command, s_Swerve);
  private SendableChooser<Command> autoChooser;



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
    opController.povRight().toggleOnTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));
    opController.povLeft().toggleOnTrue(s_Swerve.fieldOrientedToggle());
    opController.button(7).onTrue(s_Swerve.resetWheels()); //window looking button

    opController.axisGreaterThan(3, 0.25).whileTrue(intakeSubsystem.runIntakeCommand());
    opController.axisGreaterThan(3, 0.25).whileFalse(intakeSubsystem.stopIntakeCommand());
    opController.axisGreaterThan(2, 0.25).whileTrue(intakeSubsystem.dropIntakeCommand());
    opController.axisGreaterThan(2, 0.25).whileFalse(intakeSubsystem.liftIntakeCommand());

    opController.a().onTrue(intakeSubsystem.liftIntakeCommand());
    opController.b().onTrue(intakeSubsystem.dropIntakeCommand());
    // opController.povUp().onTrue(LL_sub.autoAlignCommand());

    //shooter Controls
    shootController.a().onTrue(shooter_sub.slowShooter());
    shootController.y().onTrue(shooter_sub.fastShooter());
    shootController.x().onTrue(shooter_sub.stopShooter());
    shootController.b().onTrue(shooter_sub.speedUpCommand());
    shootController.axisGreaterThan(3, 0.25).whileTrue(shooter_sub.shootSpeedBreach());
    shootController.axisGreaterThan(2, 0.25).whileTrue(shooter_sub.indexSpeedBreach());
    shootController.axisGreaterThan(3, 0.25).whileFalse(shooter_sub.stopBreach());
    shootController.axisGreaterThan(2, 0.25).whileFalse(shooter_sub.stopBreach());


    //Auto fire Controls
    // opController.axisGreaterThan(3, 0.5).onTrue(shooter_sub.closeSpeakerCommand());


  }
}
