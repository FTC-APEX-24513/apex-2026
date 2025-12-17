package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.EPipeline

class LimelightSubsystem(hardwareMap: HardwareMap) {
    private val limelight: Limelight3A = hardwareMap.get(Limelight3A::class.java, "limelight")

    fun useAprilTagPipeline() {
        limelight.pipelineSwitch(EPipeline.APRILTAG.ordinal)
    }
}