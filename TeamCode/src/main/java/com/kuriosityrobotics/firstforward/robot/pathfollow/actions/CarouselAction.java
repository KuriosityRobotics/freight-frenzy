package com.kuriosityrobotics.firstforward.robot.pathfollow.actions;

import com.kuriosityrobotics.firstforward.robot.Robot;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class CarouselAction extends Action {
    private static final long WAIT_TIME_MS = 3000;

    @Override
    public void tick(Robot robot) {
        super.tick(robot);

        if (this.msSinceStart() >= WAIT_TIME_MS) {
            robot.carouselModule.spin = false;
            this.completed = true;
        } else {
            robot.carouselModule.spin = true;
        }
    }
}
