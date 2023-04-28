package org.firstinspires.ftc.teamcode.LastSeason.Commands;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="LEDTeleOp")

public class LEDTeleOp extends OpMode {
    LED led;
    @Override
    public void init() {
        led = new LED();
        led.init(hardwareMap, telemetry);
        led.showTimers();
    }

    @Override
    public void start() {
        led.startTimers();
    }

    @Override
    public void loop() {
        led.checkDeadlines();
    }
}
