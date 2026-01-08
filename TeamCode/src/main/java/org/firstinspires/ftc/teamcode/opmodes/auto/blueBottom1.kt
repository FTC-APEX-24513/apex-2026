package org.firstinspires.ftc.teamcode.opmodes.auto

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

    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer

    private var pathState = 0
    private var actionState = 0

    private lateinit var intake: IntakeSubsystem
    private lateinit var transfer: TransferSubsystem
    private lateinit var outtake: OuttakeSubsystem
    private lateinit var spindexer: SpindexerSubsystem
    private lateinit var limelight: LimelightSubsystem

    private val startPoseBlueBottom = Pose(56.0, 16.0, Math.toRadians(180.0))
    private val blueBottomRowStart = Pose(40.0, 36.0, Math.toRadians(180.0))
    private val blueBottomRowEnd = Pose(20.0, 36.0, Math.toRadians(180.0))
    private val blueScore = Pose(48.0, 48.0, Math.toRadians(180.0))

    private lateinit var blueStartToScore: PathChain
    private lateinit var blueScoreToBottom: PathChain
    private lateinit var blueBottomIntake: PathChain
    private lateinit var blueBottomToScore: PathChain

    override fun init() {
        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()

        follower = Constants.createFollower(hardwareMap)

        intake = IntakeSubsystem(hardwareMap)
        transfer = TransferSubsystem(hardwareMap)
        outtake = OuttakeSubsystem(hardwareMap)
        spindexer = SpindexerSubsystem(hardwareMap)
        limelight = LimelightSubsystem(hardwareMap)

        buildPaths()
        follower.setStartingPose(startPoseBlueBottom)
    }

    override fun start() {
        setPathState(0)
    }

    override fun loop() {
        follower.update()
        autonomousPathUpdate()
        shootingSequence()
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
                intake.collect()
                follower.followPath(blueStartToScore)
                setPathState(1)
            }
            1 -> if (!follower.isBusy) {
                intake.stop()
                limelight.useAprilTagPipeline()
                actionTimer.resetTimer()
                actionState = 0
                setPathState(2)
            }
            2 -> if (!follower.isBusy) {
                follower.followPath(blueScoreToBottom)
                setPathState(3)
            }
            3 -> if (!follower.isBusy) {
                intake.collect()
                follower.followPath(blueBottomIntake)
                setPathState(4)
            }
            4 -> if (!follower.isBusy) {
                intake.stop()
                follower.followPath(blueBottomToScore)
                actionTimer.resetTimer()
                actionState = 0
                setPathState(5)
            }
            5 -> { /* Shooting handled in shootingSequence() */ }
            6 -> { /* Autonomous complete */ }
        }
    }

    private fun shootingSequence() {
        if (pathState != 5) return

        when (actionState) {
            0 -> if (actionTimer.getElapsedTime() > 500) { actionTimer.resetTimer(); actionState = 1 }
            1 -> if (actionTimer.getElapsedTime() > 350) { transfer.transfer(); actionTimer.resetTimer(); actionState = 2 }
            2 -> if (actionTimer.getElapsedTime() > 600) { outtake.launch(); actionTimer.resetTimer(); actionState = 3 }
            3 -> if (actionTimer.getElapsedTime() > 300) { spindexer.switch(); actionTimer.resetTimer(); actionState = 4 }
            4 -> if (actionTimer.getElapsedTime() > 350) { transfer.transfer(); actionTimer.resetTimer(); actionState = 5 }
            5 -> if (actionTimer.getElapsedTime() > 600) { outtake.launch(); actionTimer.resetTimer(); actionState = 6 }
            6 -> if (actionTimer.getElapsedTime() > 300) { spindexer.switch(); actionTimer.resetTimer(); actionState = 7 }
            7 -> if (actionTimer.getElapsedTime() > 350) { transfer.transfer(); actionTimer.resetTimer(); actionState = 8 }
            8 -> if (actionTimer.getElapsedTime() > 600) { setPathState(6) }
        }
    }

    private fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }
}
