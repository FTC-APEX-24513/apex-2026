package edu.exeter.apex.ftc.teamcode.opmodes

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

@Autonomous(name = "blueBottom_1", group = "Examples")
class BlueBottom1 : OpMode() {

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
    private val blueBottomRowStart = Pose(40.0, 36.0, Math.toRadians(180.0))
    private val blueBottomRowEnd = Pose(20.0, 36.0, Math.toRadians(180.0))
    private val blueScore = Pose(48.0, 48.0, Math.toRadians(180.0)) // tune later

    private lateinit var blueStartToScore: PathChain
    private lateinit var blueScoreToBottom: PathChain
    private lateinit var blueBottomIntake: PathChain
    private lateinit var blueBottomToScore: PathChain

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
        shootingSequence()

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
            .setLinearHeadingInterpolation(startPoseBlueBottom.heading, blueScore.heading)
            .build()

        blueScoreToBottom = follower.pathBuilder()
            .addPath(Path(BezierLine(blueScore, blueBottomRowStart)))
            .setLinearHeadingInterpolation(blueScore.heading, blueBottomRowStart.heading)
            .build()

        blueBottomIntake = follower.pathBuilder()
            .addPath(Path(BezierLine(blueBottomRowStart, blueBottomRowEnd)))
            .setLinearHeadingInterpolation(blueBottomRowStart.heading, blueBottomRowEnd.heading)
            .build()

        blueBottomToScore = follower.pathBuilder()
            .addPath(Path(BezierLine(blueBottomRowEnd, blueScore)))
            .setLinearHeadingInterpolation(blueBottomRowEnd.heading, blueScore.heading)
            .build()
    }

    private fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                // Start → first scoring position
                intake.collect() // Intake while driving
                follower.followPath(blueStartToScore)
                setPathState(1)
            }

            1 -> {
                // Reached first score
                if (!follower.isBusy) {
                    intake.stop()
                    limelight.useAprilTagPipeline()
                    actionTimer.resetTimer()
                    actionState = 0
                    setPathState(2)
                }
            }

            2 -> {
                // Move to bottom row
                if (!follower.isBusy) {
                    follower.followPath(blueScoreToBottom)
                    setPathState(3)
                }
            }

            3 -> {
                // Bottom row intake
                if (!follower.isBusy) {
                    intake.collect()
                    follower.followPath(blueBottomIntake)
                    setPathState(4)
                }
            }

            4 -> {
                // Finished bottom intake → return to score
                if (!follower.isBusy) {
                    intake.stop()
                    follower.followPath(blueBottomToScore)
                    actionTimer.resetTimer()
                    actionState = 0 // start shooting sequence
                    setPathState(5)
                }
            }

            5 -> {
                // Shooting sequence at final score
                if (!follower.isBusy) {
                    // Handled in shootingSequence()
                }
            }

            6 -> {
                // Autonomous complete
            }
        }
    }

    /** Multi-cycle shooting sequence with 3-ball spindexer */
    private fun shootingSequence() {
        if (pathState != 5) return

        when (actionState) {
            0 -> { // Localization delay
                if (actionTimer.getElapsedTime() > 500) { // 500 ms
                    actionTimer.resetTimer()
                    actionState = 1
                }
            }

            1 -> { // Transfer 1
                transfer.transfer()
                if (actionTimer.getElapsedTime() > 350) {
                    actionTimer.resetTimer()
                    actionState = 2
                }
            }

            2 -> { // Outtake 1
                outtake.launch()
                if (actionTimer.getElapsedTime() > 600) {
                    actionTimer.resetTimer()
                    actionState = 3
                }
            }

            3 -> { // Spindexer advance 1
                spindexer.switch()
                if (actionTimer.getElapsedTime() > 300) {
                    actionTimer.resetTimer()
                    actionState = 4
                }
            }

            4 -> { // Transfer 2
                transfer.transfer()
                if (actionTimer.getElapsedTime() > 350) {
                    actionTimer.resetTimer()
                    actionState = 5
                }
            }

            5 -> { // Outtake 2
                outtake.launch()
                if (actionTimer.getElapsedTime() > 600) {
                    actionTimer.resetTimer()
                    actionState = 6
                }
            }

            6 -> { // Spindexer advance 2
                spindexer.switch()
                if (actionTimer.getElapsedTime() > 300) {
                    actionTimer.resetTimer()
                    actionState = 7
                }
            }

            7 -> { // Transfer 3
                transfer.transfer()
                if (actionTimer.getElapsedTime() > 350) {
                    actionTimer.resetTimer()
                    actionState = 8
                }
            }

            8 -> { // Outtake 3
                outtake.launch()
                if (actionTimer.getElapsedTime() > 600) {
                    setPathState(6) // Auto done
                }
            }
        }
    }

    private fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }
}
