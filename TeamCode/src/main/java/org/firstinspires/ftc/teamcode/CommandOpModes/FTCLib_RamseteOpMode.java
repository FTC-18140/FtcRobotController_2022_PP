package org.firstinspires.ftc.teamcode.CommandOpModes;

import com.arcrobotics.ftclib.command.RamseteCommand;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffOdometrySubsystem;

import java.util.ArrayList;

@Autonomous(name = "Ramsete Auto")
public class FTCLib_RamseteOpMode extends TBDOpModeBase
{
    ChassisSubsystem chassis = null;
    DiffOdometrySubsystem odometry = null;

    @Override
    public void init()
    {
        try
        {
            chassis = new ChassisSubsystem(hardwareMap, telemetry);
            register( chassis );

            odometry = new DiffOdometrySubsystem( chassis::getLeftEncoderDistance,
                                                  chassis::getRightEncoderDistance,
                                                  telemetry );
            register( odometry );

            // Create Trajectory
            Pose2d trajStart = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
            Pose2d trajEnd = new Pose2d(40, 0, Rotation2d.fromDegrees(0));

            ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
            interiorWaypoints.add(new Translation2d(10, 0));
            interiorWaypoints.add(new Translation2d(20, 0));

            TrajectoryConfig config = new TrajectoryConfig(30, 30);
            Trajectory theTraj = TrajectoryGenerator.generateTrajectory(
                    trajStart,
                    interiorWaypoints,
                    trajEnd,
                    config);

            RamseteController theController = new RamseteController(2, 0.7);
            DifferentialDriveKinematics theDDKinem = new DifferentialDriveKinematics(odometry.TRACK_WIDTH);

            RamseteCommand step1 = new RamseteCommand(theTraj, odometry::getPose, theController, theDDKinem, chassis::tankDrive);

        }
        catch (Exception e)
        {
            telemetry.addData("Something did not initialize properly.", 0);
            telemetry.addData("Ugh: ", "%s, %s, %s", e.getStackTrace()[1], e.getStackTrace()[2], e.getStackTrace()[3]);
        }
    }

    @Override
    public void init_loop()
    {
        if (odometry != null) { odometry.update(); }
        telemetry.addData("Init Loop is running... ", 1);
        telemetry.addData("X:  ", odometry.getPose().getX());
        telemetry.addData("Y:  ", odometry.getPose().getY());
        telemetry.addData("HDG:  ", odometry.getPose().getHeading());
    }

    @Override
    public void start()
    {
    }

    @Override
    public void stop()
    {
        super.stop();
        chassis.stop();
    }
}
