package frc.robot.utils;

import frc.robot.Constants.PositionValueConstants;

public class ShoulderRegression {
    public static double [][] kShoulderManualCounts = {
        //x is distance from target (m)
        //y is shoulder angle (raw encoder counts)
        {1.27, 0},
        {1.71, 0.1},
        {2.61, 0.13},
        {3.215, 0.175},
        {3.825, 0.195}
    };

    public static double distanceToShoulderCounts(double meters){
        var command = -(1.04334/(1+Math.pow(Math.E, 0.84*(meters+0.075)))) + 0.2614;
        if(meters<PositionValueConstants.minDistanceFromSpeaker){
            return 0;
        }
        else if (command<PositionValueConstants.minShoulderPos){
            return 0;
        }
        else if (command>PositionValueConstants.maxShoulderPos){
            return 0;
        }
        else{
        return command;
        }
    }
}
