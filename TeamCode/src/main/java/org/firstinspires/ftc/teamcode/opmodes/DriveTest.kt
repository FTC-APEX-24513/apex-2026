package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.max


@TeleOp(name = "drivetest")
class DriveTest : LinearOpMode() {
    // Declare OpMode members for each of the 4 motors.
    private val runtime = ElapsedTime()
    private var frontLeft: DcMotor? = null
    private var backLeft: DcMotor? = null
    private var frontRight: DcMotor? = null
    private var backRight: DcMotor? = null

    override fun runOpMode() {
        frontLeft = hardwareMap.get(DcMotor::class.java, "leftFront").apply {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        };
        backLeft = hardwareMap.get(DcMotor::class.java, "leftRear").apply {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        };
        frontRight = hardwareMap.get(DcMotor::class.java, "rightFront").apply {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        };  
        backRight = hardwareMap.get(DcMotor::class.java, "rightRear").apply {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        };

        frontLeft!!.direction = DcMotorSimple.Direction.REVERSE
        backLeft!!.direction = DcMotorSimple.Direction.REVERSE
        frontRight!!.direction = DcMotorSimple.Direction.REVERSE
        backRight!!.direction = DcMotorSimple.Direction.FORWARD

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()
        runtime.reset()

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            var max: Double

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            val axial = -gamepad1.left_stick_y.toDouble() // Note: pushing stick forward gives negative value
            val lateral = gamepad1.left_stick_x.toDouble()
            val yaw = gamepad1.right_stick_x.toDouble()

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            var frontLeftPower = axial + lateral + yaw
            var frontRightPower = axial - lateral - yaw
            var backLeftPower = axial - lateral + yaw
            var backRightPower = axial + lateral - yaw

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = max(abs(frontLeftPower), abs(frontRightPower))
            max = max(max, abs(backLeftPower))
            max = max(max, abs(backRightPower))

            if (max > 1.0) {
                frontLeftPower /= max
                frontRightPower /= max
                backLeftPower /= max
                backRightPower /= max
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            frontLeft!!.power = frontLeftPower
            frontRight!!.power = frontRightPower
            backLeft!!.power = backLeftPower
            backRight!!.power = backRightPower

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: $runtime")
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower)
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower)
            telemetry.update()
        }
    }
}

