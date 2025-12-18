//package org.firstinspires.ftc.teamcode.opmodes
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import com.qualcomm.robotcore.hardware.DcMotor
//import com.qualcomm.robotcore.hardware.Servo
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem
//
//
//@TeleOp
//class BallShooterTeleop : OpMode() {
//    private val intakeSubsystem = IntakeSubsystem(hardwareMap)
//    private val outtakeSubsystem = OuttakeSubsystem(hardwareMap)
//    private val transferSubsystem = TransferSubsystem(hardwareMap)
//    private val spindexerSubsystem = SpindexerSubsystem(hardwareMap)
//    private val servoIncrement = 0.005f
//
//    private var trianglePress = false
//
//
//    override fun init() {
//        intakeMotor = hardwareMap.get<DcMotor?>(DcMotor::class.java, "intake")
//        intakeMotor!!.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE))
//        flywheel = hardwareMap.get<DcMotor?>(DcMotor::class.java, "flywheel")
//        flywheel!!.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
//        transfer = hardwareMap.get<Servo?>(Servo::class.java, "transfer")
//        spindexer = hardwareMap.get<Servo?>(Servo::class.java, "spindexer")
//        transfer!!.setPosition(0.2639)
//        gamepad1.setLedColor(195.0, 247.0, 202.0, 10)
//    }
//
//    override fun loop() {
//        if (gamepad1.square) {
//            intake()
//        } else {
//            intakeMotor!!.setPower(0.0)
//        }
//
//        if (gamepad1.dpad_up) {
//            transfer(0f)
//        }
//        if (gamepad1.dpad_down) {
//            transfer(0.2639f)
//        }
//
//        if (gamepad1.triangle) {
//            flywheel(0.75f)
//        } else {
//            trianglePress = false
//        }
//        if (gamepad1.circle) {
//            flywheel(0f)
//        } else {
//            trianglePress = false
//        }
//
//        if (gamepad1.dpad_left) {
//            spindexer(0f)
//        }
//        if (gamepad1.dpad_right) {
//            spindexer(.22f)
//        }
//
//        telemetry.addData("transfer pos", transfer!!.getPosition())
//        telemetry.update()
//
//        telemetry.addData("spindex pos", spindexer!!.getPosition())
//        telemetry.update()
//    }
//
//    private fun intake() {
//        intakeMotor!!.setPower(1.0)
//    }
//
//    private fun transfer(servoDirection: Float) {
//        transfer!!.setPosition(servoDirection.toDouble())
//    }
//
//    private fun spindexer(servoDirection: Float) {
//        spindexer!!.setPosition(servoDirection.toDouble())
//    }
//
//    private fun flywheel(flywheelDirection: Float) {
//        if (!trianglePress) {
//            flywheel!!.setPower(flywheelDirection.toDouble())
//            trianglePress = true
//        }
//    }
//}