package com.kuriosityrobotics.configuration;

public class TestModule implements Module{
    @Config(configName = "a")
    public static double coeff;
}
