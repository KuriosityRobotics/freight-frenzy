package com.kuriosityrobotics.firstforward.robot.modules;

import android.os.SystemClock;
import android.util.Log;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class OuttakeModule implements Module, Telemeter {
    //constants
    private static final double HOPPER_EXTENDED = 0.7;
    private static final double HOPPER_RETRACTED = 0;
    private static final double HOPPER_IN = 0.906181;
    private static final double HOPPER_DUMP = 0;
    private static final double HOPPER_HOLD = 0.65;
    //TODO determine actual marking
    private static final int SLIDE_UP = -330; //encoder marking
    private static final int SLIDE_DOWN = 0;   //encoder marking
    //ACTIONS
    //time constants
    //TODO determine time
    private static final long HOPPER_EXTEND_TIME = 500;
    private static final long HOPPER_ROTATE_TIME = 950;
    private static final long HOPPER_DUMP_TIME = 400;
    private static final long SLIDE_RAISE_TIME = 350;
    private final boolean isOn = true;
    //states
    public boolean extendHopper = false;
    public double rotateHopper = HOPPER_IN;
    public boolean dumpHopper = false;
    public int slideTargetPos = 0;
    private boolean hopperIsIn = true;
    private long doingActionFor = 0;
    private long lastActionTime = 0;
    public boolean currentlyDoingAction = false;

    //servos
    Servo linkage;
    Servo pivot;
    Servo hopper;
    //motors
    DcMotor slide;

    State extendHopperAction = new State(() -> linkage.setPosition(HOPPER_EXTENDED), HOPPER_EXTEND_TIME);
    State retractHopperAction = new State(() -> linkage.setPosition(HOPPER_RETRACTED), HOPPER_EXTEND_TIME);
    State moveHopperBackAction = new State(() -> pivot.setPosition(HOPPER_IN), HOPPER_ROTATE_TIME);
    State dumpHopperAction = new State(() -> hopper.setPosition(HOPPER_DUMP), HOPPER_DUMP_TIME);
    State holdHopperAction = new State(() -> hopper.setPosition(HOPPER_HOLD), 1);
    State raiseSlideAction = new State(() -> slide.setTargetPosition(SLIDE_UP), SLIDE_RAISE_TIME);
    State lowerSlideAction = new State(() -> slide.setTargetPosition(SLIDE_DOWN), SLIDE_RAISE_TIME);

    ArrayList<State> actions = new ArrayList<>() {{
        add(raiseSlideAction);
        add(extendHopperAction);
        add(dumpHopperAction);
        add(holdHopperAction);
        add(retractHopperAction);
        add(lowerSlideAction);
    }};
    StateExecutor placeFreightOnTopAction = new StateExecutor(actions);
    //time tracking
    private long previousTime = 0;
    private long timeExtendingFor = 0;
    private boolean fullyExtended = false;
    private long timeRetractingFor = 0;
    private boolean fullyRetracted = true;
    private long timeDumpingFor = 0;
    private boolean hopperDumped = false;
    private long timeRotatingFor = 0;

    public OuttakeModule(Robot robot) {

        robot.telemetryDump.registerTelemeter(this);

        linkage = robot.getServo("linkage_servo");
        pivot = robot.getServo("pivot_servo");
        hopper = robot.getServo("hopper_pivot_servo");

        slide = robot.getDcMotor("linear_slide_motor");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
    }

    public void update() {
        placeFreightOnTop();
        updateHopperPositions();

        //actions and interference
        //slide - none
        //hopper - none
        //extending - none
        //retracting - has to be hopper in
        //rotating other  - has to be extend out
        //rotating in - none
        if (!currentlyDoingAction) {
            slide.setTargetPosition(slideTargetPos);

            if (extendHopper) {
                linkage.setPosition(HOPPER_EXTENDED);
                if (fullyExtended) {
                    pivot.setPosition(rotateHopper);
                }
            } else {
                double positionPlaceholder = rotateHopper;

                rotateHopper = HOPPER_IN;
                pivot.setPosition(HOPPER_IN);
                if (hopperIsIn) {
                    rotateHopper = positionPlaceholder;
                    linkage.setPosition(HOPPER_RETRACTED);
                }
            }

            if (dumpHopper) {
                hopper.setPosition(HOPPER_DUMP);
            } else {
                hopper.setPosition(HOPPER_HOLD);
            }
        }
        /*
        if (!currentlyDoingAction){
            slide.setTargetPosition(slideTargetPos);

            if (dumpHopper){
                hopper.setPosition(HOPPER_DUMP);
            }else{
                hopper.setPosition(HOPPER_HOLD);
            }

            if (extendHopper){
                linkage.setPosition(HOPPER_EXTENDED);
                if (fullyExtended){
                    pivot.setPosition(rotateHopper);
                }
            }else{
                rotateHopper = HOPPER_IN;
                pivot.setPosition(HOPPER_IN);
                if (hopperIsIn){
                    linkage.setPosition(HOPPER_RETRACTED);
                }
            }
        }
        //Log.i("outtake", "servo pos: " + rotateHopper);

         */
    }

    public void updateHopperPositions() {
        long currentTime = SystemClock.elapsedRealtime();
        long timeChange = currentTime - previousTime;

        if (extendHopper) {
            timeExtendingFor += timeChange;
            timeRetractingFor = 0;
        } else {
            timeRetractingFor += timeChange;
            timeExtendingFor = 0;
        }
        if (rotateHopper == HOPPER_IN) {
            timeRotatingFor += timeChange;
        } else {
            timeRotatingFor = 0;
        }
        if (dumpHopper) {
            timeDumpingFor += timeChange;
        } else {
            timeDumpingFor = 0;
        }

        fullyExtended = timeExtendingFor > HOPPER_EXTEND_TIME;
        fullyRetracted = timeRetractingFor > HOPPER_EXTEND_TIME;
        hopperIsIn = timeRotatingFor > HOPPER_ROTATE_TIME;
        hopperDumped = timeDumpingFor > HOPPER_DUMP_TIME;

        previousTime = currentTime;
    }

    public void placeFreightOnTop() {
        long currentTime = SystemClock.elapsedRealtime();

        if (currentlyDoingAction && doingActionFor <= placeFreightOnTopAction.getTotalTime()) {
            placeFreightOnTopAction.update();
            //Log.i("outtake", "UPDATED");
            doingActionFor += (currentTime - lastActionTime);
        } else {
            placeFreightOnTopAction.reset();

            doingActionFor = 0;
            currentlyDoingAction = false;
        }

        lastActionTime = currentTime;
        //Log.i("outtake",  "doing action: " + currentlyDoingAction);
        Log.i("outtake", "doing action for: " + doingActionFor);
    }


    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public String getName() {
        return "OuttakeModule";
    }

    @Override
    public Iterable<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("linkage pos: " + extendHopper);
        data.add("pivot pos: " + rotateHopper);
        data.add("hopper pivot pos: " + dumpHopper);

        return data;
    }
}
