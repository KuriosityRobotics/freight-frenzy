package com.kuriosityrobotics.firstforward.robot.modules;

public interface Module {
    void update();

    default void onStart() {}

    default void onClose() {}

    boolean isOn();

    String getName();

    /**
     * @return maximum update frequency of this module in hertz (0 for no limit)
     */
    default int maxFrequency() {
        return 0;
    }
}