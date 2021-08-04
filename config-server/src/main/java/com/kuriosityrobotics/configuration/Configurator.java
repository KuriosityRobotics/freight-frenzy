package com.kuriosityrobotics.configuration;

import com.moandjiezana.toml.Toml;
import org.reflections.Reflections;
import org.reflections.scanners.FieldAnnotationsScanner;
import org.reflections.util.ClasspathHelper;
import org.reflections.util.ConfigurationBuilder;
import org.reflections.util.FilterBuilder;
import spark.Filter;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Map;
import java.util.stream.Collectors;

import static spark.Spark.*;

public class Configurator {
    private static final String CONFIG_FOLDER_NAME = "configurations";
    private static final String CONFIG_FOLDER_PREFIX = CONFIG_FOLDER_NAME + "/";

    /**
     * It is possible that a non-static field could be annotated with the @Config annotation.  In this case, you need to manually
     * pass Object instances to configure, as they are dynamically created and can't be as easily discovered as in static methods.
     * The only main use case of this is probably for setting default values in instances.
     * @param configPath
     * @param modules
     */
    public static void loadConfig(String configPath, Object modules[]) {
        Map<String, Object> toml = new Toml().read(new File(configPath)).toMap();

        for (Object module : modules) {
            Arrays.stream(module.getClass().getDeclaredFields()).filter(n -> n.isAnnotationPresent(Config.class)).forEach(field -> {
                try {
                    field.set(module, toml.get(field.getAnnotation(Config.class).configName()));
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                }
            });
        }
    }


    /**
     * The configurator is capable of automatically loading config variables for static fields annotated with the @Config annotation.
     * Specify the base package name (for us, com.kuriosityrobotics works fine), and pass in the config location.
     * You can run this right when the JVM starts up, and it will have no performance impact once the load is complete.
     * @param configPath A path to a TOML file to get the config values from.
     * @param packageName The base package to discover configurable classes in.
     */
    public static void loadConfigFieldsStatic(String configPath, String packageName) {
        var fields = new Reflections(new ConfigurationBuilder()
                .setUrls(ClasspathHelper.forPackage(packageName))
                .setScanners(new FieldAnnotationsScanner())
                .filterInputsBy(new FilterBuilder().includePackage(packageName))

        ).getFieldsAnnotatedWith(Config.class);
        Map<String, Object> toml = new Toml().read(new File(configPath)).toMap();

        for (Field field : fields) {
            try {
                field.set(null, toml.get(field.getAnnotation(Config.class).configName()));
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }

        }
    }

    public static void main(String[] args) throws IOException {
        loadConfigFieldsStatic("configurations/mainconfig.toml", "com.kuriosityrobotics");
        System.out.println(new TestModule().coeff);
        runServer(new TestModule());
    }

    /**
     * This runs a server on port 4567 which allows you to modify config files and hot reload them through a web interface.
     * The server is optional, and the loadConfig and loadConfigFieldsStatic methods will work regardless of whether it's running or not.
     * @param args
     * @throws IOException
     */
    public static void runServer(Object... args) throws IOException {
        new File("configurations").mkdir();
        if (!new File("configurations/mainconfig.toml").exists())
            new File("configurations/mainconfig.toml").createNewFile();


        loadConfig("configurations/mainconfig.toml", args);

        staticFiles.location("/build");
        get("/configurations", (req, res) ->
                Arrays.stream(new File(CONFIG_FOLDER_NAME).listFiles()).map(File::getName).collect(Collectors.joining(","))
        );
        get("/configurations/:name", (req, res) -> {
            try {
                return Files.readAllBytes(Path.of(CONFIG_FOLDER_PREFIX + req.params("name")));
            } catch (Exception e) {
                return "";
            }
        });
        post("/configurations/:name/save", (req, res) -> {
            System.out.println(req.body());
            Files.writeString(Path.of(CONFIG_FOLDER_PREFIX + req.params("name")), req.body());
            return String.format("Updated config %s.", req.params("name"));
        });
        post("/configurations/:name/activate", (req, res) -> {
            String configName = CONFIG_FOLDER_PREFIX + req.params("name");
            loadConfig(configName, args);
            loadConfigFieldsStatic("configurations/mainconfig.toml", "com.kuriosityrobotics");
            return String.format("(re)loaded config %s.", configName);
        });

        after((Filter) (request, response) -> {
            response.header("Access-Control-Allow-Origin", "*");
            response.header("Access-Control-Allow-Methods", "*");
        });


    }
}
