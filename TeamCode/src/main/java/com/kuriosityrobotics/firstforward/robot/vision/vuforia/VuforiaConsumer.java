package com.kuriosityrobotics.firstforward.robot.vision.vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Vuforia consumer interface
 *
 * @author Alec
 */
public interface VuforiaConsumer {
    /**
     * Set up the Vuforia
     *
     * @param vuforia
     */
    void setup(VuforiaLocalizer vuforia);

    /**
     * Update position
     */
    void update();
}
