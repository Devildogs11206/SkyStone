package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous
public class aBlue2DriveUnderBridge extends LinearOpMode {
    protected Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);

        robot.init();

        waitForStart();

        robot.drive(.5 ,0,8);
        robot.turn(90,.5);
        robot.drive(.5,0,34);
    }


}
