package com.kuriosityrobotics.firstforward.robot.util;

public class Button {
    private boolean previous = false;

    private boolean state = false;

    public boolean isToggled(boolean input) {
        if (!previous && input) {
            state = !state;
        }

        previous = input;

        return state;
    }

    public boolean isSelected(boolean input) {
        boolean ret = false;
        if (!previous && input) {
            ret = true;
        }
        previous = input;
        return ret;
    }
}
