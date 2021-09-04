package com.kuriosityrobotics.firstforward.robot.modules;

public interface Module {

    void initModules();

    default boolean initAsync() {
        return true;
    }

    default void onStart() {
    }

    default void onClose() {
    }

    void update();

    boolean isOn();

    String getName();
}