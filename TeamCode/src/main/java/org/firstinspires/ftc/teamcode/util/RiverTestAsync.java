package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Thunderbot_2022;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous
public class RiverTestAsync extends OpMode{


    Thunderbot_2022 robot = new Thunderbot_2022();

    SampleMecanumDrive drive;
    enum State {
        START,
        TRAJECTORY_1,
        LIFT_1,
        CLAW_1,
        FORWARD_1,
        CLAW_2,
        LIFT_2,
        BACK_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        IDLE
    }

    Trajectory trajectory_1;

    Trajectory forward_1;
    Trajectory back_1;

    Trajectory trajectory_2;

    Trajectory trajectory_3;
    boolean liftDone = false;
    State currentState = State.START;

    @Override
    public void init(){robot.init(hardwareMap, telemetry);}

    @Override
    public void start(){
        drive = new SampleMecanumDrive(hardwareMap);

        trajectory_1 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .lineToSplineHeading(new Pose2d(20, 0, Math.toRadians(-45)))
                .build();

        forward_1 = drive.trajectoryBuilder(trajectory_1.end())
                .forward(10)
                .build();

        back_1 = drive.trajectoryBuilder(trajectory_1.end())
                .back(10)
                .build();

        trajectory_2 = drive.trajectoryBuilder(trajectory_1.end())
                .splineTo(new Vector2d(10,-10), Math.toRadians(-180))
                .build();
    }

    @Override
    public void loop(){
        switch (currentState) {
            case START:
                if(!drive.isBusy()) {
                    currentState = State.TRAJECTORY_1;
                    drive.followTrajectoryAsync(trajectory_1);
                }
                break;

            case TRAJECTORY_1:
                if(!drive.isBusy()) {
                    currentState = State.LIFT_1;
                }
                break;

            case LIFT_1:
                if (!liftDone) {
                    liftDone = robot.armstrong.liftUpDistance(5, 0.4);

                }else{
                    currentState = State.CLAW_1;
                    liftDone = false;
                }
                break;

            case CLAW_1:
                if (!liftDone) {
                    liftDone = robot.armstrong.clawMove(robot.armstrong.getCLAW_MIN());

                }else{
                    currentState = State.FORWARD_1;
                    drive.followTrajectory(forward_1);
                }
                break;

            case FORWARD_1:
                if(!drive.isBusy()) {
                    currentState = State.CLAW_2;
                    liftDone = false;
                }
                break;

            case CLAW_2:
                if (!liftDone) {
                    liftDone = robot.armstrong.clawMove(robot.armstrong.getCLAW_MAX());

                }else{
                    currentState = State.LIFT_2;
                    liftDone = false;
                }
                break;

            case LIFT_2:
                if (!liftDone) {
                    liftDone = robot.armstrong.liftUpDistance(10, 0.4);

                }else{
                    currentState = State.BACK_1;
                    drive.followTrajectory(back_1);
                }
                break;

            case BACK_1:
                if(!drive.isBusy()) {
                    currentState = State.TRAJECTORY_2;
                    drive.followTrajectory(trajectory_2);
                }
                break;

            case TRAJECTORY_2:
                if(!drive.isBusy()) {
                    currentState = State.IDLE;
                }
                break;

            case IDLE:
                telemetry.addData("done", "true");
                break;
            default:
                break;
        }

        telemetry.addData("SlidePosition", robot.armstrong.getLiftPosition());


        drive.update();
        robot.update();


    }

}
