plugins {
    `java-library`
    `maven-publish`
}

group = "com.basicmotor"
version = "1.2.0"

val wpilibVersion = "2025.3.2"
val advantageKitVersion = "4.1.2"
val phoenix6Version = "25.4.0"
val revLibVersion = "2025.0.3"

java {
    toolchain {
        languageVersion.set(JavaLanguageVersion.of(17))
    }
    withJavadocJar()
    withSourcesJar()
}


publishing {
    publications {
        create<MavenPublication>("mavenJava") {
            from(components["java"])
            groupId = "com.basicmotor"
            artifactId = "basic-motor"
            version = version
        }
    }
}

sourceSets {
    main {
        java {
            srcDirs("src/main/java")
            exclude("**/example_projects/**")
        }
    }
}

repositories {
    mavenCentral()
    mavenLocal()
    maven {
        url = uri("https://frcmaven.wpi.edu/artifactory/release")
    }
    maven {
        url = uri("https://frcmaven.wpi.edu/artifactory/littletonrobotics-mvn-release")
    }
    maven{
        url = uri("https://maven.ctr-electronics.com/release/")
    }
    maven{
        url = uri("https://maven.revrobotics.com/")
    }
}

dependencies {
    implementation("edu.wpi.first.wpilibj:wpilibj-java:${wpilibVersion}")
    implementation("edu.wpi.first.wpimath:wpimath-java:${wpilibVersion}")
    implementation("edu.wpi.first.wpiutil:wpiutil-java:${wpilibVersion}")
    implementation("edu.wpi.first.wpiunits:wpiunits-java:${wpilibVersion}")
    implementation("org.littletonrobotics.akit:akit-java:${advantageKitVersion}")
    implementation("com.ctre.phoenix6:wpiapi-java:${phoenix6Version}")
    implementation(("com.revrobotics.frc:REVLib-java:${revLibVersion}"))
    implementation("us.hebi.quickbuf:quickbuf-runtime:1.3.3")

    testImplementation(platform("org.junit:junit-bom:5.10.0"))
    testImplementation("org.junit.jupiter:junit-jupiter")
}

tasks.test {
    useJUnitPlatform()
}

tasks.named<Jar>("sourcesJar") {
    duplicatesStrategy = DuplicatesStrategy.EXCLUDE
}