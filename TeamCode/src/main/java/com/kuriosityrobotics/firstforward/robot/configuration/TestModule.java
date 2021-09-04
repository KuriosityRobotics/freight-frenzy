package com.kuriosityrobotics.firstforward.robot.configuration;

import com.kuriosityrobotics.firstforward.robot.modules.Module;

public class TestModule implements Module {
    @Config(configName = "testmodule.coeffa")
    public static double coeffA;

    @Override
    public void initModules() {

    }

    @Override
    public void update() {

    }

    @Override
    public boolean isOn() {
        return false;
    }

    @Override
    public String getName() {
        return null;
    }
}
