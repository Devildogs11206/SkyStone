/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


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

    public int cameraMonitorViewId;

    public Position position = new Position();
    public Orientation rotation = new Orientation();

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    HardwareMap hardwareMap = null;
    private ElapsedTime period  = new ElapsedTime();

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
 }
