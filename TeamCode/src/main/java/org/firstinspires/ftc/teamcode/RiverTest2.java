package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class RiverTest2 extends LinearOpMode {

    Thunderbot_2022 robot = new Thunderbot_2022();
    public final int DIST = 20;
    SampleMecanumDrive drive = null;



    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);
        Trajectory step1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(DIST,0), 0)
                .build();

        Trajectory step2 = drive.trajectoryBuilder(step1.end())
                .splineTo(new Vector2d(DIST,DIST), 90)
                .build();

        Trajectory step3 = drive.trajectoryBuilder(step1.end())
                .splineTo(new Vector2d(0,DIST), 180)
                .build();

        Trajectory step4 = drive.trajectoryBuilder(step1.end())
                .splineTo(new Vector2d(0,0), 180)
                .build();

        Trajectory circle = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(DIST,0), 0)
                .splineTo(new Vector2d(DIST,DIST), 90)
                .splineTo(new Vector2d(0,DIST), 180)
                .splineTo(new Vector2d(0,0), -90)
                .build();
        waitForStart();
        /*
        drive.followTrajectory(step1);
        drive.followTrajectory(step2);
        drive.followTrajectory(step3);
        drive.followTrajectory(step4);
        */
        drive.followTrajectory(circle);
    }




}
