package frc.robot.commands;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShoulderSubsystem;

public class BumpShoulderValueCommand extends Command{
    //instantiate stuff
    ShoulderSubsystem m_shoulderSubsystem;
    String m_direction;

    public BumpShoulderValueCommand(ShoulderSubsystem shoulderSubsystem, String direction){
        //definitions and setting parameters equal to members
        m_shoulderSubsystem = shoulderSubsystem;
        m_direction = direction;
        addRequirements(m_shoulderSubsystem);
    }

    @Override
    public void initialize() {
    }
        
    @Override
    public void execute() {

        if(m_direction == "up"){
            m_shoulderSubsystem.bumpUp();
        }

        if(m_direction == "down"){
            m_shoulderSubsystem.bumpDown();
        }
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return true;
    }

}