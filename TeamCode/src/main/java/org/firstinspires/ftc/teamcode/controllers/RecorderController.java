package org.firstinspires.ftc.teamcode.controllers;

import com.google.gson.reflect.TypeToken;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecorderState.IDLE;
import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecorderState.RECORDING;
import static org.firstinspires.ftc.teamcode.controllers.RecorderController.RecorderState.REPLAYING;

public class RecorderController extends RobotController {
    public enum RecorderState{
        IDLE,
        RECORDING,
        REPLAYING
    }

    private String filename = "recorder-data.json";

    private RecorderState state = IDLE;

    private List<Gamepad> gamepads = null;

    public RecorderController(OpMode opMode){super(opMode);}

    @Override
    public void execute(){
        switch(state){
            case IDLE:
                if(gamepad1.start){enterRecording();}
                if(gamepad1.a){enterReplaying();}
                break;
            case RECORDING:
                if(gamepad1.back){enterIdle();}
                else{record();}
                break;
            case REPLAYING:
                if(gamepad1.back){enterIdle();}
                else if(gamepads.size() != 0){replay();}
                else{enterIdle();}
                break;
        }

        telemetry.addData("Recording State", state);
        telemetry.addData("Gamepad States",gamepads == null ? 0 : gamepads.size());
        telemetry.addLine();
    }

    private void record(){

        Gamepad gamepad1copy = new Gamepad();
        Gamepad gamepad2copy = new Gamepad();

        try {
            gamepad1copy.copy(gamepad1);
            gamepad2copy.copy(gamepad2);
        } catch (Exception e) {
            // TODO: Handle this exception later.
        }

        gamepads.add(gamepad1copy);
        gamepads.add(gamepad2copy);

    }

    private void replay(){
        Gamepad gamepad1Copy = gamepads.remove(0);
        Gamepad gamepad2Copy = gamepads.remove(0);

        try {
            gamepad1.copy(gamepad1Copy);
            gamepad2.copy(gamepad2Copy);
        }
        catch (Exception e) {

        }
    }

    private void enterIdle(){
        if(state == RECORDING){save();}
        state = IDLE;
    }

    private void enterRecording(){
        gamepads = new ArrayList<>();
        state = RECORDING;
    }

    private void enterReplaying(){
        state = REPLAYING;
        load();
    }

    private void save(){
        String json = SimpleGson.getInstance().toJson(gamepads.toArray());
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file,json);
    }
    private void load(){
        try{
            File file = AppUtil.getInstance().getSettingsFile(filename);
            String json = ReadWriteFile.readFileOrThrow(file);
            gamepads = SimpleGson.getInstance().fromJson(json, new TypeToken<List<Gamepad>>(){}.getType());
        }catch(Exception e){
            telemetry.addData("Error","An error occured attempting to load the playback data"+e.toString());
        }
    }
}
