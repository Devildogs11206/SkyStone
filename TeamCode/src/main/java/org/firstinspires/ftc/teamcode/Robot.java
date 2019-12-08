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
    private DcMotor right_rear;
    private DcMotor right_front;
    private DcMotor slide;
    private DcMotor lift;
    private DcMotor tilt;

    private Servo claw;


    public double liftStrength = 0.2;


    public Robot(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_rear = hardwareMap.get(DcMotor.class,"left_rear");
        right_rear = hardwareMap.get(DcMotor.class, "right_rear");
        right_front = hardwareMap.get(DcMotor.class,"right_front");

        left_front.setDirection(DcMotor.Direction.REVERSE);
        left_rear.setDirection(DcMotor.Direction.REVERSE);
        right_rear.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_front.setPower(0);
        left_rear.setPower(0);
        right_rear.setPower(0);
        right_front.setPower(0);

        slide = hardwareMap.get(DcMotor.class,"slide");
        slide.setPower(0);

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setPower(0);

        tilt = hardwareMap.get(DcMotor.class, "tilt");
        tilt.setPower(0);

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
        lift.setPower(power);
    }

    String liftStatus;

    int maxPos = 1000;
    int minPos = 0;


    public void lift(double power){

        int position = lift.getCurrentPosition();

        if((power>0 && position< maxPos) || (power<0 && position> minPos)){

            lift.setPower(power);
            liftStatus = "Lift in motion";
            telemetry.addData("Lift Status",liftStatus);
            telemetry.update();

        }
        else{
            lift.setPower(0);
            liftStatus = "Limit exceeded";
            telemetry.addData("Lift Status",liftStatus);
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
