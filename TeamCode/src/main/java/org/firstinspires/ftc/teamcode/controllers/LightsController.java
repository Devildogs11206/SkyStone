package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.internal.VisionThread;
import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BLACK;

public class LightsController extends RobotController {
    public LightsController(OpMode opMode) {
        super(opMode);
    }

    public void execute(){

        if (gamepad2.back){robot.setLights(BLACK);}

    }
}
