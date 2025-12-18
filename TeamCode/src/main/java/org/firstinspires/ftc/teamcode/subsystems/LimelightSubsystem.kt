package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.EPipeline

class LimelightSubsystem(hardwareMap: HardwareMap) {
    private val limelight: Limelight3A = hardwareMap.get(Limelight3A::class.java, "limelight").also {
        it.setPollRateHz(90)
        it.pipelineSwitch(EPipeline.APRILTAG.ordinal)
        it.start()
    }

    fun useAprilTagPipeline() {
        limelight.pipelineSwitch(EPipeline.APRILTAG.ordinal)
    }
}