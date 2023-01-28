package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.LiftSubsystem;

public class LiftDistanceCommand extends CommandBase
{
    private final double myDistance;
    private final double mySpeed;
    private final LiftSubsystem theSubsystem;


    LiftDistanceCommand(double distance, double speed, LiftSubsystem armstrong)
    {
        myDistance = distance;
        mySpeed = speed;
        theSubsystem = armstrong;
    }

    @Override
    public void initialize() {
        if (myDistance > 0)
        {
            theSubsystem.liftUp(mySpeed);
        }
        else
        {
            theSubsystem.liftDown(mySpeed);
        }
     }

    @Override
    public void end(boolean interrupted) {
        theSubsystem.liftStop();
    }


    @Override
    public boolean isFinished() {
        return Math.abs(theSubsystem.getAverageEncoderDistance()) >= myDistance;
    }


}
