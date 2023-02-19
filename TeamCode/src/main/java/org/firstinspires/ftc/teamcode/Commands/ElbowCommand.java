package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

public class ElbowCommand extends CommandBase
{
    private final double myAngle;
    private final ArmSubsystem myArm;

    public ElbowCommand(double angle, ArmSubsystem arm)
    {
        myAngle = angle;
        myArm = arm;
        addRequirements(myArm);
    }

    @Override
    public void initialize()
    {
        if (myArm != null )
        {
            myArm.elbowMove(myAngle);
        }
    }

}
