import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

plugins {
    alias(libs.plugins.android.library)
    alias(libs.plugins.kotlin.android)
}

android {
    namespace = "com.qualcomm.ftcrobotcontroller"

    compileSdk = 30

    defaultConfig {
        minSdkVersion(24)
        //noinspection ExpiredTargetSdkVersion
        targetSdkVersion(28)
        buildConfigField(
            "String", "APP_BUILD_TIME", '"' + (SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSSZ", Locale.ROOT).format(
                Date()
            )) + '"'
        )
    }

    buildFeatures {
        buildConfig = true
    }



    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
    kotlinOptions {
        jvmTarget = "11"
    }

}

dependencies {

    implementation(libs.core.ktx)
    implementation(libs.androidx.appcompat)
    testImplementation(libs.junit)
    androidTestImplementation(libs.ext.junit)

    implementation(libs.ftc.inspection)
    implementation(libs.ftc.blocks)
    implementation(libs.ftc.robotcore)
    implementation(libs.ftc.robotserver)
    implementation(libs.ftc.onbotjava)
    implementation(libs.ftc.hardware)
    implementation(libs.ftc.ftccommon)
    implementation(libs.ftc.vision)

}
