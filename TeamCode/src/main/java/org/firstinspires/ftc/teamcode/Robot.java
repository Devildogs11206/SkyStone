package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    private DcMotor left_front;
    private DcMotor left_rear;
    private DcMotor right_front;
    private DcMotor right_rear;
    private DcMotor slide;
    private DcMotor lift;
    private DcMotor tilt;

    private Servo claw;

    public Robot(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_front.setDirection(DcMotor.Direction.FORWARD);

        left_rear = hardwareMap.get(DcMotor.class,"left_rear");
        left_rear.setDirection(DcMotor.Direction.FORWARD);

        right_front = hardwareMap.get(DcMotor.class,"right_front");
        right_front.setDirection(DcMotor.Direction.REVERSE);

        right_rear = hardwareMap.get(DcMotor.class, "right_rear");
        right_rear.setDirection(DcMotor.Direction.REVERSE);

        slide = hardwareMap.get(DcMotor.class,"slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tilt = hardwareMap.get(DcMotor.class, "tilt");

        claw = hardwareMap.get(Servo.class, "claw");
    }

    public void drive (double left, double right){
        left_front.setPower(left);
        left_rear.setPower(left);
        right_rear.setPower(right);
        right_front.setPower(right);
    }

    public void slide(double power){
        slide.setPower(power);
    }

    public void tilt(double power) {
        tilt.setPower(power);
    }

    String liftStatus;
    int minPos = 0;
    int maxPos = 1000;

    public void lift(double power){
        int position = lift.getCurrentPosition();

        if((power > 0 && position < maxPos) || (power < 0 && position > minPos)){
            lift.setPower(power);
            liftStatus = "Lift in motion";
            telemetry.addData("Lift Status", liftStatus);
            telemetry.update();
        } else {
            lift.setPower(0);
            liftStatus = "Limit exceeded";
            telemetry.addData("Lift Status", liftStatus);
            telemetry.update();
        }
    }

    public void openClaw(){
        claw.setPosition(0.7);
    }

    public void closeClaw(){
        claw.setPosition(1);
    }
}
