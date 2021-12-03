package com.kuriosityrobotics.firstforward.robot.modules;

import android.os.SystemClock;

import java.util.ArrayList;

public class StateExecutor {
    private ArrayList<State> states;
    private State currentState;

    private int currentStep = -1;
    private double totalSteps;
    private long completeTime =  0;

    public StateExecutor(ArrayList<State> states){
        this.states = states;
        totalSteps = states.size();
    }

    public void update(){
        if (currentStep < totalSteps){
            long currentTime = SystemClock.elapsedRealtime();

            if (currentTime >= completeTime){
                currentStep++;
                currentState = states.get(currentStep);
                completeTime = currentTime + currentState.getBlockDuration();
            }

            currentState.apply();
        }
    }

    public void reset(){
        currentStep = -1;
        completeTime = 0;
    }

    public long getTotalTime(){
        long time = 0;
        for (State state : states){
            time += state.getBlockDuration();
        }
        return time;
    }
}
