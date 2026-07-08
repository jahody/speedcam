pluginManagement {
    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}

dependencyResolutionManagement {
    repositories {
        google()
        mavenCentral()
    }
}

rootProject.name = "speedcam-android"

// Pure-JVM estimation math + unit tests; always buildable, no Android SDK needed
include(":core")

// The app module requires an Android SDK (Android Studio provides one via
// local.properties). Machines without an SDK can still run :core tests.
val hasSdk = File(rootDir, "local.properties").exists() ||
    System.getenv("ANDROID_HOME") != null ||
    System.getenv("ANDROID_SDK_ROOT") != null
if (hasSdk) include(":app")
