// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.PreferenceKeys;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AdjustAngleToDistanceCommand;
import frc.robot.commands.AutoInShooterCommand;
import frc.robot.commands.AutoIntakeIndexUntilTrippedCommand;
import frc.robot.commands.BumpShoulderValueCommand;
import frc.robot.commands.InShooterCommand;
import frc.robot.commands.IndexToShootCommand;
import frc.robot.commands.IntakeForSecsCommand;
import frc.robot.commands.IntakeIndexUntilTrippedCommand;
import frc.robot.commands.RumbleForSecsCommand;
import frc.robot.commands.SetFlipoutCommand;
import frc.robot.commands.SetShoulderCommand;
import frc.robot.commands.SpinUpAutoCommand;
import frc.robot.commands.SpinUpShootCommand;
import frc.robot.commands.vision.AimAtSpeakerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeFlipoutSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShootSubsystem m_shootSubsystem = new ShootSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem();
  private final LEDSSubsystem m_ledsSubsystem = new LEDSSubsystem();
  //private final IntakeFlipoutSubsystem m_flipoutSubsystem = new IntakeFlipoutSubsystem();
  SendableChooser<Command> autoChooser;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  // controllers
  CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(ControllerConstants.kOperatorControllerPort);

  // commands
  SequentialCommandGroup handoffCommand = new SequentialCommandGroup(
    new IntakeIndexUntilTrippedCommand(m_intakeSubsystem, m_indexerSubsystem, m_shootSubsystem),
    new RumbleForSecsCommand(1, m_driverController).alongWith(
    new InShooterCommand(m_intakeSubsystem, m_indexerSubsystem))
  );
  SequentialCommandGroup autoHandoffCommand = new SequentialCommandGroup(
    new IntakeIndexUntilTrippedCommand(m_intakeSubsystem, m_indexerSubsystem, m_shootSubsystem),
    new AutoInShooterCommand(m_intakeSubsystem, m_indexerSubsystem)
  );

  SequentialCommandGroup autoHandoffCommand2 = new SequentialCommandGroup(
    new AutoIntakeIndexUntilTrippedCommand(m_intakeSubsystem, m_indexerSubsystem),
    new AutoInShooterCommand(m_intakeSubsystem, m_indexerSubsystem)
  );


  public RobotContainer() {
    m_robotDrive.calibrateGyro();
    m_shoulderSubsystem.resetEncoder();
    // named commands configuration
    NamedCommands.registerCommand("Flipout", /*new SetFlipoutCommand(m_flipoutSubsystem, "out")*/ new WaitCommand(0.001));
    NamedCommands.registerCommand("Run Intake", autoHandoffCommand);
    //NamedCommands.registerCommand("Intake 0.5", new IntakeForSecsCommand(m_intakeSubsystem, 0.9));
    NamedCommands.registerCommand("Intake 0.5", autoHandoffCommand2);
    NamedCommands.registerCommand("Spin Up Shoot", new SpinUpShootCommand(m_shootSubsystem, m_indexerSubsystem, m_intakeSubsystem));
    NamedCommands.registerCommand("Spin Up", new SpinUpAutoCommand(m_shootSubsystem));
    NamedCommands.registerCommand("Shoot", new IndexToShootCommand(m_shootSubsystem, m_indexerSubsystem, m_intakeSubsystem));
    // default commands
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.kDriveDeadband),
                -MathUtil.applyDeadband(-m_driverController.getLeftX(), ControllerConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), ControllerConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
    m_shootSubsystem.setDefaultCommand(new RunCommand(() -> m_shootSubsystem.manual(m_driverController), m_shootSubsystem));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoMode", m_chooser);

    //auto options
        m_chooser.addOption("Score 1 wherever", new SpinUpShootCommand(m_shootSubsystem, m_indexerSubsystem, m_intakeSubsystem));

        //m_chooser.addOption("score 2 center (amp)", m_robotDrive.getAuto("Score 2 Center"));
        //m_chooser.addOption("score 2 center (middle)", m_robotDrive.getAuto("Score 2 Center Middle Note"));
        //m_chooser.addOption("score 2 center (source)", m_robotDrive.getAuto("Score 2 Center Source Note"));
        //m_chooser.addOption("score 2 wall side (amp)", m_robotDrive.getAuto("Score 2 Wall Side"));
        //m_chooser.addOption("score 2 field side (source)", m_robotDrive.getAuto("Score 2 Field Side"));

        //m_chooser.addOption("score 3 center (amp, middle)", m_robotDrive.getAuto("Score 3 Center"));
        //m_chooser.addOption("score 3 wall side (amp, middle)", m_robotDrive.getAuto("Score 3 Wall Side"));
        //m_chooser.addOption("score 3 field side (source, middle)", m_robotDrive.getAuto("Score 3 Field Side"));

        m_chooser.addOption("score 4 center NO FLIPOUT (amp, middle, source)", m_robotDrive.getAuto("Score 4 Center"));
        m_chooser.addOption("score 4 center NO FLIPOUT (source, middle, amp)", m_robotDrive.getAuto("Score 4 Center Inverted"));
        m_chooser.addOption("score 4 center NO FLIPOUT (middle, amp, source)", 
            m_robotDrive.getAuto("Score 4 Center Start Middle Note"));
        m_chooser.addOption("score 4 center start middle note (middle, source, amp)", 
            m_robotDrive.getAuto("Score 4 Center Start Middle Note Inverted"));
        m_chooser.addOption("score 4 center (amp, middle, source) flipout", m_robotDrive.getAuto("Score 4 Center Flipout"));


        m_chooser.addOption("score 4 FLIPOUT (amp, middle, midline 1)", m_robotDrive.getAuto("Score 3 Midline 1 Center Flipout"));
        m_chooser.addOption("score 4 FLIPOUT (amp, middle, midline 2)", m_robotDrive.getAuto("Score 3 Midline 2 Center Flipout"));
        m_chooser.addOption("score 4 FLIPOUT ampside 1, 2 midline (subwoof side)", m_robotDrive.getAuto("Score 3 Midline 1, 2 Amp Wall Flipout"));

        //m_chooser.addOption("score 4 wall side (amp, middle, source)", m_robotDrive.getAuto("Score 4 Wall Side"));
        //m_chooser.addOption("score 4 field side (amp, middle, source)", m_robotDrive.getAuto("Score 4 Field Side"));

        m_chooser.addOption("score 5?? NO FLIPOUT", m_robotDrive.getAuto("Score 5 Center"));
        m_chooser.addOption("score 5 FLIPOUT, midline amp side", m_robotDrive.getAuto("Score 5 Center Flipout Midline 1"));
        m_chooser.addOption("score 5 FLIPOUT, midline 2nd from amp side",
            m_robotDrive.getAuto("Score 5 Center Flipout Midline 2"));
        m_chooser.addOption("score 5 FLIPOUT, midline middle (amp, middle, source)",
            m_robotDrive.getAuto("Score 5 Center Flipout Midline 3"));
        m_chooser.addOption("score 5 FLIPOUT, midline middle (middle, amp, source)", 
            m_robotDrive.getAuto("Score 5 Center Flipout Midline 3 Middle Amp Source"));
        m_chooser.addOption("score 5 FLIPOUT midline middle (middle, source, amp)",
          m_robotDrive.getAuto("Score 5 Center Flipout Midline 3 Middle Source Amp"));
        m_chooser.addOption("score 5 FLIPOUT midline middle (source, middle, amp)", 
            m_robotDrive.getAuto("Score 5 Center Flipout Midline 3 Inverse"));
      

        m_chooser.addOption("null auto", new WaitCommand(0.05));




    /*SmartDashboard.putData(new InstantCommand(() -> m_robotDrive.resetEstimator(new Pose2d())));
    
    // Button for disabling filtering
    SmartDashboard.putData("Disable vision filtering for 1 second",
                           Commands.sequence(
                             m_robotDrive.runOnce(() -> m_robotDrive.getVisionDataProvider().setUseFiltering(false)),
                             Commands.waitSeconds(1),
                             m_robotDrive.runOnce(() -> m_robotDrive.getVisionDataProvider().setUseFiltering(true))
                           ));

    // Initialize some PID-tuning preferences
    Preferences.initDouble(PreferenceKeys.kAutomaticTurningP, frc.robot.Constants.PIDConstants.kDefaultAutomaticTurningP);
    Preferences.initDouble(PreferenceKeys.kAutomaticTurningI, frc.robot.Constants.PIDConstants.kDefaultAutomaticTurningI);
    Preferences.initDouble(PreferenceKeys.kAutomaticTurningD, frc.robot.Constants.PIDConstants.kDefaultAutomaticTurningD);
    */
    configureBindings();
  }

  private void configureBindings() {
    /*new JoystickButton(m_driverController.getHID(), ControllerConstants.kVisionTurnButton)
        .whileTrue(new AimAtSpeakerCommand(m_robotDrive, m_driverController.getHID()));*/

    //shoulder presets
    new JoystickButton(m_driverController.getHID(), ControllerConstants.shoulderHomeButton).whileTrue(
      new SetShoulderCommand(m_shoulderSubsystem, "home"));

    new JoystickButton(m_driverController.getHID(), ControllerConstants.shoulderAmpButton).whileTrue(
      new SetShoulderCommand(m_shoulderSubsystem, "amp"));

    new JoystickButton(m_driverController.getHID(), ControllerConstants.shoulderProtButton).whileTrue(
      new SetShoulderCommand(m_shoulderSubsystem, "protected"));

    /*new JoystickButton(m_driverController.getHID(), ControllerConstants.kVisionTurnButton).whileTrue(
      new AdjustAngleToDistanceCommand(m_shoulderSubsystem));

    //regression testing stuff
    new JoystickButton(m_operatorController.getHID(), ControllerConstants.bumpUpButton).whileTrue(
      new BumpShoulderValueCommand(m_shoulderSubsystem, "up")
    );

    new JoystickButton(m_operatorController.getHID(), ControllerConstants.bumpDownButton).whileTrue(
      new BumpShoulderValueCommand(m_shoulderSubsystem, "down")
    );
*/
    //handoff
    new Trigger(m_driverController.axisGreaterThan(ControllerConstants.intakeAxis, 0.5))
      .whileTrue(handoffCommand);

    //flipout
    /*new Trigger(m_driverController.axisGreaterThan(ControllerConstants.intakeFlipoutAxis, 0.5))
      .toggleOnTrue(Commands.startEnd(m_flipoutSubsystem::setOut, m_flipoutSubsystem::setIn,
         m_flipoutSubsystem));*/

    new JoystickButton(m_driverController.getHID(), ControllerConstants.shootButton)
      .whileTrue(
        new InstantCommand(() -> m_indexerSubsystem.runFast(), m_indexerSubsystem)
        .alongWith(
          new InstantCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem)))
      .whileFalse(
        new InstantCommand(() -> m_indexerSubsystem.stop(), m_indexerSubsystem)
        .alongWith(
          new InstantCommand(() -> m_intakeSubsystem.stop(), m_intakeSubsystem)
      ));

  }

  public void autonomousInit(){
    m_robotDrive.setUsingVision(false);
  }

  public void teleopInit(){
    m_robotDrive.setUsingVision(false);
  }

  public Command getAutonomousCommand() {
      return m_chooser.getSelected();   
    } 

  }
