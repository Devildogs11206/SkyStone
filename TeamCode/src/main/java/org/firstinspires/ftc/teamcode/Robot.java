package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    private HardwareMap hardwareMap;

    private DcMotor left_drive_0;
    private DcMotor left_drive_1;
    private DcMotor right_drive_2;
    private DcMotor right_drive_3;
    private DcMotor slide_0;
    private DcMotor lift_1;

    private Servo claw_0;

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        left_drive_0 = hardwareMap.get(DcMotor.class, "left_drive_0");
        left_drive_1 = hardwareMap.get(DcMotor.class,"left_drive_1");
        right_drive_2 = hardwareMap.get(DcMotor.class, "right_drive_2");
        right_drive_3 = hardwareMap.get(DcMotor.class,"right_drive_3");

        left_drive_0.setDirection(DcMotor.Direction.REVERSE);
        left_drive_1.setDirection(DcMotor.Direction.REVERSE);
        right_drive_2.setDirection(DcMotor.Direction.FORWARD);
        right_drive_3.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left_drive_0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_drive_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive_3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_drive_0.setPower(0);
        left_drive_1.setPower(0);
        right_drive_2.setPower(0);
        right_drive_3.setPower(0);

        slide_0 = hardwareMap.get(DcMotor.class,"slide_0");
        slide_0.setPower(0);

        lift_1 = hardwareMap.get(DcMotor.class, "lift_1");
        lift_1.setPower(0);

        claw_0 = hardwareMap.get(Servo.class, "claw_0");
    }

    public void drive (double left, double right){
        left_drive_0.setPower(left);
        left_drive_1.setPower(left);
        right_drive_2.setPower(right);
        right_drive_3.setPower(right);
    }

    public void slide(double power){
        slide_0.setPower(power);
    }

    public void lift(double power) {
        lift_1.setPower(power);
    }

    public void openClaw(){
        claw_0.setPosition(0.7);
    }

    public void closeClaw(){
        claw_0.setPosition(1);
    }
}
