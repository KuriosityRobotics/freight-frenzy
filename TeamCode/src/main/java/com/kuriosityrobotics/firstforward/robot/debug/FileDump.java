package com.kuriosityrobotics.firstforward.robot.debug;

import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;
import android.util.Pair;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.util.Date;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;

public class FileDump {
    private static boolean activated = false;
    private static final Set<Pair<Field, Object>> dataFields = ConcurrentHashMap.newKeySet();
    private static final ConcurrentHashMap<Field, Object> previousValues = new ConcurrentHashMap<>();

    private static PrintWriter writer;
    private static long startTime;

    public static void activate() {
        try {
            File file = new File(AppUtil.ROBOT_DATA_DIR + "/" + new Date().getTime() + ".csv");
            writer = new PrintWriter(file);

            System.out.printf("Started dumping to %s.%n", file.getAbsolutePath());
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }

        dataFields.stream()
                .map(n -> n.first)
                .forEach(n -> n.setAccessible(true));

        writer.println(
                dataFields.stream()
                        .map(n -> n.first)
                        .map(Field::getName)
                        .collect(Collectors.joining(",")));
        startTime = SystemClock.currentThreadTimeMillis();

        activated = true;
    }

    public static void addField(String fieldName, Object receiver) {
        try {
            dataFields.add(new Pair<>(receiver.getClass().getDeclaredField(fieldName), receiver));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    public static void addVisionReplay(Bitmap replay) {
        File file = new File(AppUtil.ROBOT_DATA_DIR + "/" + "webcam-frame-%d.jpg", String.valueOf(new Date().getTime()));
        try {
                try (FileOutputStream outputStream = new FileOutputStream(file)) {
                    replay.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
                    System.out.println(String.format("Started dumping to %s.", file.getName()));
                }
        } catch (IOException e) {
            Log.v("Vision Dump", String.valueOf(e));
        }
    }

    @NonNull
    @Override
    public String toString() {
        return super.toString();
    }

    public static void update() {
        if (activated) {
            boolean anyUpdated = FileDump.dataFields.stream().anyMatch(n -> {
                Field field = n.first;
                Object instance = n.second;
                Object previousValue = previousValues.getOrDefault(field, null);
                Object value;
                try {
                    field.setAccessible(true);
                    value = field.get(instance);
                    previousValues.put(field, value);
                } catch (IllegalAccessException e) {
                    throw new RuntimeException(e);
                }

                return previousValue == null || !value.equals(previousValue);
            });
            if (anyUpdated)
                writer.println(SystemClock.currentThreadTimeMillis() - startTime + "," + FileDump.dataFields.stream().map(n ->
                        {
                            Field field = n.first;
                            Object instance = n.second;

                            try {
                                field.setAccessible(true);
                                Object value = field.get(instance);
                                return value.toString();
                            } catch (IllegalAccessException e) {
                                throw new RuntimeException(e);
                            }
                        }
                ).collect(Collectors.joining(",")));
        }
    }
}
