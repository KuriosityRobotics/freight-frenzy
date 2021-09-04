package com.kuriosityrobotics.firstforward.robot.modules;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;

public abstract class ModuleCollection implements Module {
    protected Module[] modules;

    @Override
    public void initModules() {
        for (Module module : modules) {
            if (module.isOn()) {
                module.initModules();
            }
        }
    }

    ArrayList<Module> toInitModules = new ArrayList<>();

    @Override
    public boolean initAsync() {
        if (toInitModules.isEmpty()) {
            toInitModules = new ArrayList<>(Arrays.asList(modules));
        }

        Iterator<Module> itr = toInitModules.iterator();
        while (itr.hasNext()) {
            Module module = itr.next();
            if (!module.isOn()) {
                itr.remove();
            } else if (module.initAsync()) {
                itr.remove();
            }
        }

        return toInitModules.isEmpty();
    }

    @Override
    public void onStart() {
        for (Module module : modules) {
            if (module.isOn()) {
                module.onStart();
            }
        }
    }

    @Override
    public void onClose() {
        for (Module module : modules) {
            try {
                module.onClose();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void update() {
        for (Module module : modules) {
            if (module.isOn()) {
                module.update();
            }
        }
    }
}

