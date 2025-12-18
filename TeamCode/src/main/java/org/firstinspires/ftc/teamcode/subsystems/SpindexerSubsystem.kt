package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class SpindexerSubsystem(hardwareMap: HardwareMap) {
    private val servo: Servo = hardwareMap.servo.get("spindexer").apply{
        position = 0.2639;
    }
}