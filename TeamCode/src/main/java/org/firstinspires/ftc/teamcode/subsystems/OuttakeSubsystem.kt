package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap

class OuttakeSubsystem(hardwareMap: HardwareMap) {
    private val motor: DcMotor = hardwareMap.dcMotor.get("intake").apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    };

    fun launch() {
        motor.power = 0.9
    }
}