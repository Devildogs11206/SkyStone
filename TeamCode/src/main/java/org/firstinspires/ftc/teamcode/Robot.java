package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.List;


public class Robot {
    /* Public OpMode members. */
    public DcMotor    left_drive_0   = null;
    public DcMotor    left_drive_2   = null;
    public DcMotor    right_drive_1  = null;
    public DcMotor    right_drive_3  = null;

    public DcMotor    lift_0         = null;
    public DcMotor    slide          = null;

    public Servo      claw_0         = null;

    public WebcamName webcamName;

    public List<Recognition> recognitions = new ArrayList<>();

    public int cameraMonitorViewId;
    public int tfodMonitorViewId;

    public Position position = new Position();
    public Orientation rotation = new Orientation();

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    HardwareMap hardwareMap = null;
    private ElapsedTime period  = new ElapsedTime();

    private VisionThread visionThread;

    /* Constructor */
    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /* Initialize standard Hardware interfaces */
    public void init() {
        // Define and Initialize Motors
        left_drive_0 = hardwareMap.get(DcMotor.class, "left_drive_0");
        left_drive_2 = hardwareMap.get(DcMotor.class,"left_drive_2");
        right_drive_1 = hardwareMap.get(DcMotor.class, "right_drive_1");
        right_drive_3 = hardwareMap.get(DcMotor.class,"right_drive_3");

        left_drive_0.setDirection(DcMotor.Direction.REVERSE);
        left_drive_2.setDirection(DcMotor.Direction.REVERSE);
        right_drive_1.setDirection(DcMotor.Direction.FORWARD);
        right_drive_3.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left_drive_0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_drive_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive_3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to zero power
        left_drive_0.setPower(0);
        left_drive_2.setPower(0);
        right_drive_1.setPower(0);
        right_drive_3.setPower(0);

        lift_0 = hardwareMap.get(DcMotor.class, "lift_0");
        lift_0.setPower(0);

        claw_0 = hardwareMap.get(Servo.class, "claw_0");

        slide = hardwareMap.get(DcMotor.class,"lift_0");
        slide.setPower(0);

        webcamName = hardwareMap.get(WebcamName.class,"Webcam 1");

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId","id",hardwareMap.appContext.getPackageName());

        visionThread = new VisionThread(null,this);
        visionThread.start();
    }

    public void lift(double power) {
        lift_0.setPower(power);
    }

    public void slide(double power){
        slide.setPower(power);
    }

    public void drive (double left, double right){
        left_drive_0.setPower(left);
        left_drive_2.setPower(left);
        right_drive_1.setPower(right);
        right_drive_3.setPower(right);
    }
    public void moveClaw (double pos){

    }

    public void openClaw(){
        claw_0.setPosition(0.7);
    }

    public void closeClaw(){
        claw_0.setPosition(1);
    }

    public void stop(){
        visionThread.abort();
    }
 }
