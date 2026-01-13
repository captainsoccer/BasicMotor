plugins {
    `java-library`
    `maven-publish`
}

val phoenix6Version = "26.1.0"
val phoenix5Version = "5.36.0"

java {
//    toolchain {
//        languageVersion.set(JavaLanguageVersion.of(17)) // WPILib uses Java 17
//    }
    withSourcesJar()
    withJavadocJar()
}

repositories {
    mavenCentral()
    maven{
        url = uri("https://maven.ctr-electronics.com/release/")
    }
}

dependencies {
    implementation(project(":basicmotor-core"))

    // CTRE Phoenix dependencies
    compileOnly("com.ctre.phoenix6:wpiapi-java:${phoenix6Version}")
    compileOnly("com.ctre.phoenix:api-java:${phoenix5Version}")
    compileOnly("com.ctre.phoenix:wpiapi-java:${phoenix5Version}")
}

tasks.test {
    useJUnitPlatform()
}

publishing {
    publications {
        create<MavenPublication>("mavenJava") {
            from(components["java"])
            artifactId = "basicmotor-ctre-brushed"

            pom {
                name.set("CTRE Brushed Motor Controllers")
                description.set("ctre brushed motor controllers implementations for the BasicMotor ecosystem.")
                url.set("https://github.com/captainsoccer/BasicMotor")

                licenses {
                    license {
                        name.set("MIT License")
                        url.set("https://opensource.org/licenses/MIT")
                    }
                }
                developers {
                    developer {
                        id.set("captainsoccer")
                        name.set("BasicMotor Maintainers")
                        url.set("https://github.com/captainsoccer")
                    }
                }
                scm {
                    url.set("https://github.com/captainsoccer/BasicMotor")
                    connection.set("scm:git:https://github.com/captainsoccer/BasicMotor.git")
                    developerConnection.set("scm:git:git@github.com:captainsoccer/BasicMotor.git")
                }
            }
        }
    }

    repositories {
        // Example: local test repo
        // maven { url = uri(layout.buildDirectory.dir("repo")) }
    }
}
