package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.LiftSubsystem;

public class LiftDistanceCommand extends CommandBase
{
    private final double myDistance;
    private final double mySpeed;
    private final LiftSubsystem myLift;

    LiftDistanceCommand(double distance, double speed, LiftSubsystem lift)
    {
        myDistance = distance;
        mySpeed = speed;
        myLift = lift;
        addRequirements(myLift);
    }

    @Override
    public void initialize()
    {
        if (myDistance > 0)
        {
            myLift.liftUp(mySpeed);
        }
        else
        {
            myLift.liftDown(mySpeed);
        }
     }

    @Override
    public void end(boolean interrupted) {
        myLift.liftStop();
    }


    @Override
    public boolean isFinished()
    {
        return Math.abs(myLift.getAverageEncoderDistance()) >= myDistance;
    }
}
