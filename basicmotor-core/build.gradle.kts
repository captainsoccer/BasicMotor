plugins {
    `java-library`
    `maven-publish`
}

java {
//    toolchain {
//        languageVersion.set(JavaLanguageVersion.of(17)) // WPILib uses Java 17
//    }
    withSourcesJar()
    withJavadocJar()
}

repositories {
    mavenCentral()
}

tasks.test {
    useJUnitPlatform()
}

publishing {
    publications {
        create<MavenPublication>("mavenJava") {
            from(components["java"])
            artifactId = "basicmotor-core"

            pom {
                name.set("BasicMotor Core")
                description.set("Core interfaces, abstractions, and utilities for the BasicMotor ecosystem.")
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
}
