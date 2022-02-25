package com.kuriosityrobotics.firstforward.robot.pathfollow.actions;

import com.kuriosityrobotics.firstforward.robot.modules.carousel.CarouselModule;
import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

public class CarouselAction extends Action {
    private final CarouselModule carouselModule;
    private static final long WAIT_TIME_MS = 3000;

    public CarouselAction(CarouselModule carouselModule) {
        this.carouselModule = carouselModule;
    }

    @Override
    public void tick() {
        super.tick();

        carouselModule.maxSpeed = 0.8 * Math.PI;

        if (this.msSinceStart() >= WAIT_TIME_MS) {
            carouselModule.spin = false;
            this.completed = true;
        } else {
            carouselModule.spin = true;
        }
    }
}
