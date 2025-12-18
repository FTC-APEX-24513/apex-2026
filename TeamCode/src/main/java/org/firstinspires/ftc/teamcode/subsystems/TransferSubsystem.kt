package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class TransferSubsystem(hardwareMap: HardwareMap) {
    private val servo: Servo = hardwareMap.servo.get("transfer").apply{
        position = 0.2639;
    }

    fun transfer() {

    }
}