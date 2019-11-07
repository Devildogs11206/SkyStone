package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Claw OpMode", group="Pushbot")
public class ClawOpMode extends LinearOpMode {

    Robot robot = new Robot();

    private Servo claw_0 = null;

    //double          clawOffset      = 0;                       // Servo mid position
    //final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

    @Override
    public void runOpMode(){

        claw_0 = hardwareMap.get(Servo.class,"claw_0");

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.a){claw_0.setPosition(1);}
            if(gamepad1.b){claw_0.setPosition(0.7);}

            /*
            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad1.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad1.left_bumper)
                clawOffset -= CLAW_SPEED;


            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
            robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);


            // Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad1.y)
                robot.leftArm.setPower(robot.ARM_UP_POWER);
            else if (gamepad1.a)
                robot.leftArm.setPower(robot.ARM_DOWN_POWER);
            else
                robot.leftArm.setPower(0.0);

                */

        }

    }
}
