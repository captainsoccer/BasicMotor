plugins {
    base // keeps root free of Java source; provides clean/assemble tasks
}

val wpilibVersion = "2025.3.2"
val advantageKitVersion = "4.1.2"

// ---- Common repositories for every module ----
allprojects {
    repositories {
        mavenCentral()
        maven(url = "https://frcmaven.wpi.edu/artifactory/release/") // WPILib
        maven(url = "https://frcmaven.wpi.edu/artifactory/littletonrobotics-mvn-release") // Littleton Robotics
        // maven(url = "https://jitpack.io") // uncomment if you consume JitPack deps
    }
}

// ---- Shared config for all subprojects (modules) ----
subprojects {
    // Most modules will be Java libraries published somewhere
    apply(plugin = "java-library")
    apply(plugin = "maven-publish")

    // Group (Maven groupId). For JitPack publishing, you can set this to "com.github.captainsoccer".
    group = (rootProject.findProperty("GROUP") as String?)
        ?: "io.github.captainsoccer"

    // Version: by default each module should set its own version in its own build.gradle.kts.
    // If you want a fallback (for local dev), keep this:
    version = (findProperty("DEFAULT_VERSION") as String?) ?: "0.0.0-SNAPSHOT"

    dependencies{
        // WPILib dependencies
        add("compileOnly", "edu.wpi.first.wpilibj:wpilibj-java:${wpilibVersion}")
        add("compileOnly", "edu.wpi.first.wpimath:wpimath-java:${wpilibVersion}")
        add("compileOnly", "edu.wpi.first.wpiutil:wpiutil-java:${wpilibVersion}")
        add("compileOnly", "edu.wpi.first.wpiunits:wpiunits-java:${wpilibVersion}")
        add("compileOnly", "org.littletonrobotics.akit:akit-java:${advantageKitVersion}")
        add("compileOnly", "us.hebi.quickbuf:quickbuf-runtime:1.3.3")
    }

    the<JavaPluginExtension>().apply {
        toolchain { languageVersion.set(JavaLanguageVersion.of(17)) }
        withSourcesJar()
        withJavadocJar()
    }

//    tasks.test {
//        useJUnitPlatform()
//    }
}