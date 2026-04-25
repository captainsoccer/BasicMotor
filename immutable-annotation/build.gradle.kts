plugins {
    id("java")
}

group = "com.github.captainsoccer"
version = "6.7.1"

val javapoetVersion = "0.14.0"
val autoServiceVersion = "1.1.0"

repositories {
    mavenCentral()
}

dependencies {
    testImplementation(platform("org.junit:junit-bom:5.10.0"))
    testImplementation("org.junit.jupiter:junit-jupiter")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher")

    implementation("com.palantir.javapoet:javapoet:${javapoetVersion}")
    compileOnly("com.google.auto.service:auto-service:${autoServiceVersion}")
    annotationProcessor ("com.google.auto.service:auto-service:1.1.1")
}

tasks.test {
    useJUnitPlatform()
}