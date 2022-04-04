package com.kuriosityrobotics.firstforward.robot.modules.carousel;

import com.kuriosityrobotics.firstforward.robot.pathfollow.Action;

class CarouselAction extends Action {
    private static final long WAIT_TIME_MS = 4000;

    private final CarouselModule carouselModule;

    public CarouselAction(CarouselModule carouselModule) {
        this.carouselModule = carouselModule;
    }

    @Override
    public void tick() {
        super.tick();

        carouselModule.setMaxSpeed(0.55 * Math.PI);

        if (this.msSinceStart() >= WAIT_TIME_MS) {
            carouselModule.setSpin(false);
            this.completed = true;
        } else {
            carouselModule.setSpin(true);
            carouselModule.isSlow = true;
        }
    }
}
