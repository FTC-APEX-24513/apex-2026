package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem

@Autonomous(name = "blueBottom_0", group = "Examples")
class BlueBottom0 : OpMode() {

    // Pedro Pathing
    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer

    private var pathState = 0
    private var actionState = 0

    // Subsystems
    private lateinit var intake: IntakeSubsystem
    private lateinit var transfer: TransferSubsystem
    private lateinit var outtake: OuttakeSubsystem
    private lateinit var spindexer: SpindexerSubsystem
    private lateinit var limelight: LimelightSubsystem

    private val startPoseBlueBottom = Pose(56.0, 16.0, Math.toRadians(180.0))
    private val blueScore = Pose(48.0, 48.0, Math.toRadians(180.0)) // tune later

    private lateinit var blueStartToScore: PathChain

    override fun init() {
        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()

        follower = Constants.createFollower(hardwareMap)

        // Subsystems
        intake = IntakeSubsystem(hardwareMap)
        transfer = TransferSubsystem(hardwareMap)
        outtake = OuttakeSubsystem(hardwareMap)
        spindexer = SpindexerSubsystem(hardwareMap)
        limelight = LimelightSubsystem(hardwareMap)

        buildPaths()
        follower.setStartingPose(startPoseBlueBottom)
    }

    override fun start() {
        opmodeTimer.resetTimer()
        setPathState(0)
    }

    override fun loop() {
        follower.update()
        autonomousPathUpdate()

        telemetry.addData("Path State", pathState)
        telemetry.addData("Action State", actionState)
        telemetry.addData("X", follower.pose.x)
        telemetry.addData("Y", follower.pose.y)
        telemetry.addData("Heading", follower.pose.heading)
        telemetry.update()
    }

    private fun buildPaths() {
        blueStartToScore = follower.pathBuilder()
            .addPath(Path(BezierLine(startPoseBlueBottom, blueScore)))
            .setLinearHeadingInterpolation(
                startPoseBlueBottom.heading,
                blueScore.heading
            )
            .build()
    }

    private fun autonomousPathUpdate() {
        when (pathState) {

            0 -> {
                // Intake while driving
                intake.collect()
                follower.followPath(blueStartToScore)
                setPathState(1)
            }

            1 -> {
                if (!follower.isBusy) {
                    intake.stop()
                    limelight.useAprilTagPipeline()
                    actionTimer.resetTimer()
                    actionState = 0
                    setPathState(2)
                }
            }

            2 -> {
                // Shooting sequence
                shootingSequence()
            }

            3 -> {
                // Auto done
            }
        }
    }

    private fun shootingSequence() {
        when (actionState) {

            0 -> { // Localization delay
                if (actionTimer.elapsedTimeSeconds > 0.5) {
                    actionTimer.resetTimer()
                    actionState = 1
                }
            }

            1 -> { // Transfer 1
                transfer.transfer()
                if (actionTimer.elapsedTimeSeconds > 0.35) {
                    actionTimer.resetTimer()
                    actionState = 2
                }
            }

            2 -> { // Outtake 1
                outtake.launch()
                if (actionTimer.elapsedTimeSeconds > 0.6) {
                    actionTimer.resetTimer()
                    actionState = 3
                }
            }

            3 -> { // Switch 1
                spindexer.switch()
                if (actionTimer.elapsedTimeSeconds > 0.3) {
                    actionTimer.resetTimer()
                    actionState = 4
                }
            }

            4 -> { // Transfer 2
                transfer.transfer()
                if (actionTimer.elapsedTimeSeconds > 0.35) {
                    actionTimer.resetTimer()
                    actionState = 5
                }
            }

            5 -> { // Outtake 2
                outtake.launch()
                if (actionTimer.elapsedTimeSeconds > 0.6) {
                    actionTimer.resetTimer()
                    actionState = 6
                }
            }

            6 -> { // Switch 2
                spindexer.switch()
                if (actionTimer.elapsedTimeSeconds > 0.3) {
                    actionTimer.resetTimer()
                    actionState = 7
                }
            }

            7 -> { // Transfer 3
                transfer.transfer()
                if (actionTimer.elapsedTimeSeconds > 0.35) {
                    actionTimer.resetTimer()
                    actionState = 8
                }
            }

            8 -> { // Outtake 3
                outtake.launch()
                if (actionTimer.elapsedTimeSeconds > 0.6) {
                    setPathState(3)
                }
            }
        }
    }

    private fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }
}
