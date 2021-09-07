package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public interface VuforiaConsumer {
    void setup(VuforiaLocalizer vuforia);
    void update();
}
