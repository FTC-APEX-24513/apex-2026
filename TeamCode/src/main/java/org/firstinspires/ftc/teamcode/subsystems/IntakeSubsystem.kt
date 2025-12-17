package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap


class IntakeSubsystem(hardwareMap: HardwareMap) {
    private val motor: DcMotor = hardwareMap.dcMotor.get("intake").apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    };

    fun collect() {
        motor.power = 0.9
    }

    fun eject() {
        motor.power = -0.9
    }

    fun stop() {
        motor.power = 0.0
    }
}