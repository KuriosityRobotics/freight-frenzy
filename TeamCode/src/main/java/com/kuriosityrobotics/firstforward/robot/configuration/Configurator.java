package com.kuriosityrobotics.firstforward.robot.configuration;

import com.moandjiezana.toml.Toml;
import org.reflections.Reflections;
import org.reflections.scanners.FieldAnnotationsScanner;
import org.reflections.util.ClasspathHelper;
import org.reflections.util.ConfigurationBuilder;
import org.reflections.util.FilterBuilder;
import spark.Filter;
import spark.Spark;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;
import java.math.BigDecimal;
import java.math.BigInteger;
import java.net.MalformedURLException;
import java.net.URI;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import static spark.Spark.*;

public class Configurator {
    private static final String CONFIG_FOLDER_NAME = "configurations";
    private static final String CONFIG_FOLDER_PREFIX = CONFIG_FOLDER_NAME + "/";

    private static Object loadObject(Class<?> type, Toml config, String path) {
        if (type == long.class || type == Long.class)
            return config.getLong(path);
        else if (type == int.class || type == Integer.class)
            return config.getLong(path).intValue();
        else if (type == BigInteger.class)
            return BigInteger.valueOf(config.getLong(path));
        else if (type == float.class || type == Float.class)
            return config.getDouble(path).floatValue();
        else if (type == double.class || type == Double.class)
            return config.getDouble(path);
        else if (type == BigDecimal.class)
            return BigDecimal.valueOf(config.getDouble(path));
        else if (type == String.class)
            return config.getString(path);
        else if (type == URI.class)
            return URI.create(config.getString(path));
        else if (type == URL.class) {
            try {
                return new URL(config.getString(path));
            } catch (MalformedURLException e) {
                e.printStackTrace();
            }
        } else if (type == char.class || type == Character.class)
            return config.getString(path).charAt(0);
        else if (type == List.class)
            return config.getList(path);
        return config.to(type);
    }

    /**
     * It is possible that a non-static field could be annotated with the @Config annotation.  In this case, you need to manually
     * pass Object instances to configure, as they are dynamically created and can't be as easily discovered as in static methods.
     * The only main use case of this is probably for setting default values in instances.
     *
     * @param configPath
     * @param modules
     */
    public static void loadConfig(String configPath, Object modules[]) {
        var toml = new Toml().read(new File(configPath));

        for (Object module : modules) {
            Arrays.stream(module.getClass().getDeclaredFields()).filter(n -> n.isAnnotationPresent(Config.class)).forEach(field -> {
                try {
                    field.set(module, loadObject(field.getType(), toml, field.getAnnotation(Config.class).configName()));
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
     *
     * @param configPath  A path to a TOML file to get the config values from.
     * @param packageName The base package to discover configurable classes in.
     */
    public static void loadConfigFieldsStatic(String configPath, String packageName) {
        var fields = new Reflections(new ConfigurationBuilder()
                .setUrls(ClasspathHelper.forPackage(packageName))
                .setScanners(new FieldAnnotationsScanner())
                .filterInputsBy(new FilterBuilder().includePackage(packageName))

        ).getFieldsAnnotatedWith(Config.class);
        var toml = new Toml().read(new File(configPath));

        for (Field field : fields) {
            var value = loadObject(field.getType(), toml, field.getAnnotation(Config.class).configName());
            try {
                field.set(null, value);
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            } catch (IllegalArgumentException e) {
                System.err.printf("Value %s invalid for field %s\n", value, field.getName());
            } catch (Exception e) {
                e.printStackTrace();
            }

        }
    }

    public static void printConfigVariables(Object obj) {
        Arrays.stream(obj.getClass().getDeclaredFields()).filter(n -> n.isAnnotationPresent(Config.class)).forEach(field -> {
            try {
                System.out.printf("%s=%s\n", field.getAnnotation(Config.class).configName(), field.get(obj));
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        });
    }

    public static void main(String[] args) throws IOException {
        loadConfigFieldsStatic("configurations/mainconfig.toml", "com.kuriosityrobotics");
        System.out.println(new TestModule().coeffA);
        runServer(new TestModule());
    }

    /**
     * This runs a server on port 4567 which allows you to modify config files and hot reload them through a web interface.
     * The server is optional, and the loadConfig and loadConfigFieldsStatic methods will work regardless of whether it's running or not.
     *
     * @param args Non-static objects to configure.
     * @throws IOException
     */
    public static void runServer(Object... args) throws IOException {
        new File("configurations").mkdir();
        if (!new File("configurations/mainconfig.toml").exists())
            new File("configurations/mainconfig.toml").createNewFile();


        loadConfig("configurations/mainconfig.toml", args);
        loadConfigFieldsStatic("configurations/mainconfig.toml", "com.kuriosityrobotics");

        Spark.staticFiles.location("/build");
        Spark.get("/configurations", (req, res) ->
                Arrays.stream(new File(CONFIG_FOLDER_NAME).listFiles()).map(File::getName).collect(Collectors.joining(","))
        );
        Spark.get("/configurations/:name", (req, res) -> {
            try {
                return Files.readAllBytes(Paths.get(CONFIG_FOLDER_PREFIX + req.params("name")));
            } catch (Exception e) {
                return "";
            }
        });
        Spark.post("/configurations/:name/save", (req, res) -> {
            Files.write(Paths.get(CONFIG_FOLDER_PREFIX + req.params("name")), req.bodyAsBytes());
            return String.format("Updated config %s.", req.params("name"));
        });
        Spark.post("/configurations/:name/activate", (req, res) -> {
            String configName = CONFIG_FOLDER_PREFIX + req.params("name");
            loadConfig(configName, args);
            loadConfigFieldsStatic("configurations/mainconfig.toml", "com.kuriosityrobotics");

            for (Object arg : args)
                printConfigVariables(arg);
            return String.format("(re)loaded config %s.", configName);
        });

        Spark.after((Filter) (request, response) -> {
            response.header("Access-Control-Allow-Origin", "*");
            response.header("Access-Control-Allow-Methods", "*");
        });


    }
}
