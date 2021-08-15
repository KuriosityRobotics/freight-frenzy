package com.kuriosityrobotics.firstforward.robot.modules;

public interface Module {
    void update();

    default void onClose() {}

    boolean isOn();

    String getName();
}
