package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime


@TeleOp(name = "alignmenttest")
class AlignmentTest : LinearOpMode() {
    // Declare OpMode members for each of the 4 motors.
    private val runtime = ElapsedTime()
    private var frontLeft: DcMotor? = null
    private var backLeft: DcMotor? = null
    private var frontRight: DcMotor? = null
    private var backRight: DcMotor? = null

    override fun runOpMode() {
        frontLeft = hardwareMap.get(DcMotor::class.java, "leftFront")
        backLeft = hardwareMap.get(DcMotor::class.java, "leftRear")
        frontRight = hardwareMap.get(DcMotor::class.java, "rightFront")
        backRight = hardwareMap.get(DcMotor::class.java, "rightRear")

        frontLeft!!.direction = DcMotorSimple.Direction.REVERSE
        backLeft!!.direction = DcMotorSimple.Direction.FORWARD
        frontRight!!.direction = DcMotorSimple.Direction.REVERSE
        backRight!!.direction = DcMotorSimple.Direction.REVERSE

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()
        runtime.reset()

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.dpad_left) {
                frontLeft!!.power = 0.5;
                backLeft!!.power = 0.5;
            } else {
                frontLeft!!.power = 0.0;
                backLeft!!.power = 0.0;
            }

            if (gamepad1.dpad_right) {
                frontRight!!.power = 0.5;
                backRight!!.power = 0.5;
            } else {
                frontRight!!.power = 0.0;
                backRight!!.power = 0.0;
            }

            telemetry.addData("Status", "Run Time: $runtime")
            telemetry.addData("Front Left Power", frontLeft?.power)
            telemetry.addData("Back Left Power", backLeft?.power)
            telemetry.addData("Front Right Power", frontRight?.power)
            telemetry.addData("Back Right Power", backRight?.power)
            telemetry.update()
        }
    }
}

