package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

public class WristCommand extends CommandBase
{
    private final double myAngle;
    private final ArmSubsystem myArm;

    WristCommand(double angle, ArmSubsystem arm )
    {
        myAngle = angle;
        myArm = arm;
        addRequirements(myArm);
    }

    @Override
    public void initialize() {
        myArm.wristMove(myAngle);
    }

}
