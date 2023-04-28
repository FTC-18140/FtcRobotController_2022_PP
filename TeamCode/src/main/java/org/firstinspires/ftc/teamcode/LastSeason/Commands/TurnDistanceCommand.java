package org.firstinspires.ftc.teamcode.LastSeason.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LastSeason.Commands.CommandOpModes.AutoEpic;
import org.firstinspires.ftc.teamcode.LastSeason.Commands.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.LastSeason.Commands.Subsystems.DiffDriveOdometrySubsystem;

public class TurnDistanceCommand extends CommandBase
{
    private final ChassisSubsystem myChassisSubsystem;
    private final DiffDriveOdometrySubsystem myOdometrySubsystem;
    private final MotionProfile myMotionProfile;


    private final double turnRad;
    private double myInitialAngleRad;

    private Pose2d myRobotPose;

    private final double myTargetZoneRad;

    private final double myMaxTurnSpeed;
    private final double myMinTurnSpeed;

    private boolean myFinished = false;


    Telemetry telemetry;

    /**
     * Creates a new ArriveCommand.
     *
     */
    public TurnDistanceCommand(double turnDeg, double turnSpeed, double minTurnSpeed, double headingDecelZoneDeg, double targetBufferDeg, ChassisSubsystem chassis, DiffDriveOdometrySubsystem odometry)
    {
        turnRad = Math.toRadians(turnDeg);

        myMaxTurnSpeed = turnSpeed;
        myMinTurnSpeed = minTurnSpeed;
        myTargetZoneRad = Math.toRadians(targetBufferDeg);

        myChassisSubsystem = chassis;
        myOdometrySubsystem = odometry;
        telemetry = myChassisSubsystem.getTelemetry();

        myMotionProfile =  new MotionProfile(20, 10, headingDecelZoneDeg, 0.02);

        // temp
        myMotionProfile.telem = telemetry;

        addRequirements(myChassisSubsystem, myOdometrySubsystem);
    }

    @Override
    public void initialize()
    {
        myChassisSubsystem.setZeroBehavior(Motor.ZeroPowerBehavior.BRAKE);
        myInitialAngleRad = myOdometrySubsystem.getPose().getHeading();
    }

    @Override
    public void execute()
    {
        telemetry.addData("Executing Turn Command: ", turnRad);
        // Update the current position of the Robot by getting it from the odometry subsystem.
        myRobotPose = myOdometrySubsystem.getPose();

        // Determine if robot needs to drive and turn to get to the position.
        double[] motorPowers = new double[3];
        motorPowers[0] = 0.0;
        motorPowers[1] = 0; // no strafing

        if (turnRad > 0)
        {
            motorPowers[2] = myMaxTurnSpeed;
        }
        else
        {
            motorPowers[2] = -myMaxTurnSpeed;
        }
        telemetry.addData("Power 2: ", motorPowers[2]);

        if (AutoEpic.doLogging)
        {
            AutoEpic.logger.addField(myChassisSubsystem.getLeftEncoderDistance());
            AutoEpic.logger.addField(myChassisSubsystem.getRightEncoderDistance());
            AutoEpic.logger.addField(myOdometrySubsystem.getPose().getX());
            AutoEpic.logger.addField(myOdometrySubsystem.getPose().getY());
            AutoEpic.logger.addField(myOdometrySubsystem.getPose().getHeading());
            AutoEpic.logger.addField(motorPowers[0]);
            AutoEpic.logger.addField(motorPowers[2]);
            AutoEpic.logger.newLine();
        }

        myChassisSubsystem.arcadeDrive(motorPowers[0], motorPowers[2]);
    }

    @Override
    public void end(boolean interrupted) {
        myChassisSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(myRobotPose.getHeading() - myInitialAngleRad) >= Math.abs(turnRad);
    }
}
