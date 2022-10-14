package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class AutoTest extends OpMode {
    Thunderbot_2022 robot = new Thunderbot_2022();

    public void init() {
        robot.init(hardwareMap, telemetry);
    }

    public void start() {
        telemetry.addData("Robot Location: ", robot.updateHeading());
    }
boolean done = false;
    int state = 0;
    public void loop() {
//        if (done == false) {
//            done = robot.drive(0, 30, 0.5);

        switch (state) {
            case 0:
                if (!done) {
                    done = robot.drive(0, 40, 0.2);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 1:
                if(!done) {
                    done = robot.drive(90, 40, 0.2);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            default:
                break;
        }
    }
}
