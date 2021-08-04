
import org.gradle.jvm.tasks.Jar

plugins {
    java
    id( "com.github.johnrengelman.shadow") version "7.0.0"

}


group = "org.example"
version = "1.0-SNAPSHOT"

repositories {
    mavenCentral()
    jcenter()
}

dependencies {
    testImplementation("org.junit.jupiter:junit-jupiter-api:5.6.0")
    testRuntimeOnly("org.junit.jupiter:junit-jupiter-engine")
    implementation ("com.sparkjava:spark-core:2.9.3")
    implementation ("com.moandjiezana.toml:toml4j:0.7.2")
    implementation("org.apache.logging.log4j:log4j-core:2.14.1")
    implementation("org.apache.logging.log4j:log4j-api:2.14.1")
    implementation("org.slf4j:slf4j-api:2.0.0-alpha2")
// https://mvnrepository.com/artifact/org.slf4j/slf4j-log4j12
    implementation("org.slf4j:slf4j-log4j12:2.0.0-alpha2")
    implementation("org.reflections:reflections:0.9.12")



}

tasks.getByName<Test>("test") {
    useJUnitPlatform()
}

//sourceSets {
//    main {
//        resources {
//            srcDirs("src/main/resources")
//        }
//    }
//}

tasks {
    named<com.github.jengelman.gradle.plugins.shadow.tasks.ShadowJar>("shadowJar") {
        archiveBaseName.set("config-server")
        mergeServiceFiles()
        manifest {
            attributes(mapOf("Main-Class" to "com.kuriosityrobotics.configuration.Configurator"))
        }
    }
}
