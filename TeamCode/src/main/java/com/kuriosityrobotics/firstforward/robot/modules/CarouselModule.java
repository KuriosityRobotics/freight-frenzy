package com.kuriosityrobotics.firstforward.robot.modules;

import static java.lang.Math.PI;

import android.os.SystemClock;

import com.kuriosityrobotics.firstforward.robot.debug.telemetry.Telemeter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

public class CarouselModule implements Module, Telemeter {
    private static final double CAROUSEL_SPINNER_WHEEL_CIRCUMFERENCE = 2 * PI;
    private static final double CAROUSEL_WHEEL_CIRCUMFERENCE = 15 * PI;
    private static final double REVS_PER_CAROUSEL_REV = CAROUSEL_WHEEL_CIRCUMFERENCE / CAROUSEL_SPINNER_WHEEL_CIRCUMFERENCE;

    private final boolean isOn = true;

    //states
    public volatile boolean spin = false;
    public volatile boolean clockwise = false;
    public volatile double maxSpeed;

    private Long spinStartTimeMillis = null;

    //motors
    private DcMotorEx carouselMotor;

    public CarouselModule(HardwareMap hardwareMap) {
        this.maxSpeed = MAX_CAROUSEL_SPEED;

        carouselMotor = (DcMotorEx) hardwareMap.dcMotor.get("carousel");
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private static double MAX_CAROUSEL_SPEED = 1.0 * PI; // original:  1.4 * pi
    private static double MAX_SPEED_MS = 1300;

    private double target = 0;

    public void update() {
        if (spin) {
            if (spinStartTimeMillis == null) {
                spinStartTimeMillis = SystemClock.elapsedRealtime();
            }

            double speed = maxSpeed * Range.clip((((double)(SystemClock.elapsedRealtime() - spinStartTimeMillis)) / MAX_SPEED_MS), 0, 1);
            carouselMotor.setVelocity(clockwise ? -speed : speed, AngleUnit.RADIANS);
            target = speed;
        } else {
            spinStartTimeMillis = null;
            carouselMotor.setVelocity(0);
        }
    }

    public boolean isOn() {
        return isOn;
    }

    public String getName() {
        return "CarouselModule";
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("spin: " + spin);
        data.add("velocity: " + carouselMotor.getVelocity(AngleUnit.RADIANS));
        data.add("target: " + target);

        return data;
    }
}
