package com.kuriosityrobotics.firstforward.robot.util;

public class Toggle {
    private boolean previous = false;
    private boolean state = false;

    public boolean isToggled(boolean input) {
        if (!previous && input) {
            state = !state;
        }
        previous = input;
        return state;
    }
}