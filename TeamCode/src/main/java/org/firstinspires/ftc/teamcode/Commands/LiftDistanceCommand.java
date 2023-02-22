package org.firstinspires.ftc.teamcode.Commands;;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.LiftSubsystem;

public class LiftDistanceCommand extends CommandBase
{
    private final double myDistance;
    private final double mySpeed;
    private final LiftSubsystem myLift;
    private double myInitialDistance;

    public LiftDistanceCommand(double distance, double speed, LiftSubsystem lift)
    {
        myDistance = distance;
        mySpeed = speed;
        myLift = lift;
        addRequirements(myLift);
    }

    @Override
    public void initialize()
    {
        myInitialDistance = myLift.getAverageEncoderDistance();
    }

    @Override
    public void execute() {
          
        if (myDistance > 0)
        {
            myLift.liftUp(mySpeed);
        }
        else
        {
            myLift.liftDown(mySpeed);
        }
        myLift.telemetry.addData("Init Encoder Distance", myInitialDistance);
    }

    @Override
    public void end(boolean interrupted) {
        myLift.liftStop();
    }


    @Override
    public boolean isFinished()
    {
        return (Math.abs(myLift.getAverageEncoderDistance()- myInitialDistance) >= Math.abs(myDistance)) ||
                (myDistance < 0 && myLift.getAverageEncoderDistance() <= 0);
    }
}
