package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class RiverTest1 extends OpMode {
    Thunderbot_2022 robot = new Thunderbot_2022();
    boolean done = false;

    double power = 0.2;
    double distance = 45;
    double heading = 0;

    @Override
    public void init()
    {
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void init_loop(){}

    @Override
    public void start() {
        telemetry.addData("Robot Location: ", robot.updateHeading());
    }

    @Override
    public void loop()
    {
        if(!done){
            done = robot.gyroDrive(heading, distance, power);
        }else{
            robot.stop();
        }
    }


}
