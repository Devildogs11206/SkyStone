package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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

    private Servo claw_left;
    private Servo claw_right;

    private DigitalChannel slide_limit_front;
    private DigitalChannel slide_limit_rear;

    public Robot(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_front.setDirection(DcMotor.Direction.REVERSE);

        left_rear = hardwareMap.get(DcMotor.class,"left_rear");
        left_rear.setDirection(DcMotor.Direction.REVERSE);

        right_front = hardwareMap.get(DcMotor.class,"right_front");
        right_front.setDirection(DcMotor.Direction.FORWARD);

        right_rear = hardwareMap.get(DcMotor.class, "right_rear");
        right_rear.setDirection(DcMotor.Direction.FORWARD);

        slide = hardwareMap.get(DcMotor.class,"slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotor.Direction.REVERSE);

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotor.Direction.REVERSE);

        tilt = hardwareMap.get(DcMotor.class, "tilt");

        claw_left = hardwareMap.get(Servo.class, "claw_left");
        claw_right = hardwareMap.get(Servo.class, "claw_right");

        slide_limit_front = hardwareMap.get(DigitalChannel.class, "slide_limit_front");
        slide_limit_front.setMode(DigitalChannel.Mode.INPUT);

        slide_limit_rear = hardwareMap.get(DigitalChannel.class, "slide_limit_rear");
        slide_limit_rear.setMode(DigitalChannel.Mode.INPUT);
    }

    public void drive (double left, double right){
        left_front.setPower(left);
        left_rear.setPower(left);
        right_rear.setPower(right);
        right_front.setPower(right);
    }

    public void drive (double angle,double power,double rotation ) {
        // TODO
    }

    public void slide(double power) {
        // If the digital channel returns true it's HIGH and the button is unpressed, so we are going to ...
        boolean limitFront = !slide_limit_front.getState();
        boolean limitRear = !slide_limit_rear.getState();

        if ((power > 0 && limitRear) || (power < 0 && limitFront)) {
            slide.setPower(0);
        } else {
            slide.setPower(power);
        }
    }

    public void tilt(double power) {
        tilt.setPower(power);
    }

    public void lift(double power){
        String liftStatus;
        int minPos = 0;
        int maxPos = 8000;

        int position = lift.getCurrentPosition();

        if((power > 0 && position < maxPos) || (power < 0 && position > minPos)){
            lift.setPower(power);
            liftStatus = "Lift in motion";
        } else {
            lift.setPower(0);
            liftStatus = "Limit exceeded";
        }

        telemetry.addData("Lift Status", liftStatus);
        telemetry.addData("Lift position", position);
    }

    public void openClaw(){
        claw_left.setPosition(0.25);
        claw_right.setPosition(0.75);
        telemetry.addData("Claw Status","open");
        telemetry.addData("Claw Position","left: %.2f right: %.2f", claw_left.getPosition(), claw_right.getPosition());
    }

    public void closeClaw(){
        claw_left.setPosition(0.75);
        claw_right.setPosition(0.25);
        telemetry.addData("Claw Status","closed");
        telemetry.addData("Claw Position","left: %.2f right: %.2f", claw_left.getPosition(), claw_right.getPosition());
    }
}
