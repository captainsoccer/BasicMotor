import groovy.json.JsonSlurper
import groovy.json.JsonOutput

plugins {
    base // keeps root free of Java source; provides clean/assemble tasks
}

version = providers.gradleProperty("libVersion").get()
val versionStr = "v$version"
group   = providers.gradleProperty("group").get()

val wpilibVersion = providers.gradleProperty("wpilibVersion").get()
val advantageKitVersion = providers.gradleProperty("advantageKitVersion").get()


// ---- Common repositories for every module ----
allprojects {
    repositories {
        mavenCentral()
        maven(url = "https://frcmaven.wpi.edu/artifactory/release/") // WPILib
        maven(url = "https://frcmaven.wpi.edu/artifactory/littletonrobotics-mvn-release") // Littleton Robotics
        // maven(url = "https://jitpack.io") // uncomment if you consume JitPack deps
    }
}

tasks.register("syncVendordepsVersion") {
    group = "release"
    description = "Updates the .version in all vendordeps/*.json"
    doLast {
        val dir = file("docs")
        if (!dir.exists()) return@doLast
        dir.listFiles { f -> f.name.endsWith(".json") }?.forEach { f ->
            val obj = JsonSlurper().parse(f) as Map<*, *>
            val mutable = obj.toMutableMap()

            // 1) set the top-level version field
            mutable["version"] = version

            // 2) update nested dependency versions
            fun bumpList(key: String) {
                val list = (mutable[key] as? List<*>)?.map { it as Map<*, *> } ?: return
                val newList = list.map { dep ->
                    val dm = dep.toMutableMap()
                    if (dm["version"] is String) dm["version"] = versionStr
                    dm
                }
                mutable[key] = newList
            }
            bumpList("javaDependencies")

            // 3) write back (pretty-printed, trailing newline)
            f.writeText(JsonOutput.prettyPrint(JsonOutput.toJson(mutable)) + "\n")
            println("Updated ${f.name} -> version=$version")
        }
    }
}

// Make other tasks depend on it (so it always runs first)
tasks.named("build").configure { dependsOn("syncVendordepsVersion") }

// ---- Shared config for all subprojects (modules) ----
subprojects {
    // Most modules will be Java libraries published somewhere
    apply(plugin = "java-library")
    apply(plugin = "maven-publish")

    // Group (Maven groupId). For JitPack publishing, you can set this to "com.github.captainsoccer".
    group = rootProject.group
    // Version: by default each module should set its own version in its own build.gradle.kts.
    // If you want a fallback (for local dev), keep this:
    version = rootProject.version

    dependencies{
        // WPILib dependencies
        add("compileOnly", "edu.wpi.first.wpilibj:wpilibj-java:${wpilibVersion}")
        add("compileOnly", "edu.wpi.first.wpimath:wpimath-java:${wpilibVersion}")
        add("compileOnly", "edu.wpi.first.wpiutil:wpiutil-java:${wpilibVersion}")
        add("compileOnly", "edu.wpi.first.wpiunits:wpiunits-java:${wpilibVersion}")
        add("compileOnly", "org.littletonrobotics.akit:akit-java:${advantageKitVersion}")
        add("compileOnly", "us.hebi.quickbuf:quickbuf-runtime:1.3.3")
    }

    plugins.withType<JavaPlugin> {
        the<JavaPluginExtension>().sourceSets.configureEach {
            java {
                exclude("example_projects/**")
            }
            resources {
                exclude("example_projects/**")
            }
        }
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