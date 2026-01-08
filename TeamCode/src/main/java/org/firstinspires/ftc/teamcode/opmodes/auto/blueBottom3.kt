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

@Autonomous(name = "blueBottom_3", group = "Examples")
class BlueBottom3 : OpMode() {

    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer

    private var pathState = 0
    private var actionState = 0

    private lateinit var intake: IntakeSubsystem
    private lateinit var outtake: OuttakeSubsystem
    private lateinit var transfer: TransferSubsystem
    private lateinit var spindexer: SpindexerSubsystem
    private lateinit var limelight: LimelightSubsystem

    private val startPoseBlueBottom = Pose(56.0, 16.0, Math.toRadians(180.0))
    private val blueTopRowStart = Pose(40.0, 84.0, Math.toRadians(180.0))
    private val blueTopRowEnd = Pose(20.0, 84.0, Math.toRadians(180.0))
    private val blueMiddleRowStart = Pose(40.0, 60.0, Math.toRadians(180.0))
    private val blueMiddleRowEnd = Pose(20.0, 60.0, Math.toRadians(180.0))
    private val blueBottomRowStart = Pose(40.0, 36.0, Math.toRadians(180.0))
    private val blueBottomRowEnd = Pose(20.0, 36.0, Math.toRadians(180.0))
    private val blueScore = Pose(48.0, 48.0, Math.toRadians(180.0))

    private lateinit var blueStartToScore: PathChain
    private lateinit var blueScoreToTop: PathChain
    private lateinit var blueTopIntake: PathChain
    private lateinit var blueTopToScore: PathChain
    private lateinit var blueScoreToMiddle: PathChain
    private lateinit var blueMiddleIntake: PathChain
    private lateinit var blueMiddleToScore: PathChain
    private lateinit var blueScoreToBottom: PathChain
    private lateinit var blueBottomIntake: PathChain
    private lateinit var blueBottomToScore: PathChain

    override fun init() {
        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()

        follower = Constants.createFollower(hardwareMap)

        intake = IntakeSubsystem(hardwareMap)
        outtake = OuttakeSubsystem(hardwareMap)
        transfer = TransferSubsystem(hardwareMap)
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

        blueScoreToTop = follower.pathBuilder()
            .addPath(Path(BezierLine(blueScore, blueTopRowStart)))
            .setLinearHeadingInterpolation(blueScore.heading, blueTopRowStart.heading)
            .build()

        blueTopIntake = follower.pathBuilder()
            .addPath(Path(BezierLine(blueTopRowStart, blueTopRowEnd)))
            .setLinearHeadingInterpolation(blueTopRowStart.heading, blueTopRowEnd.heading)
            .build()

        blueTopToScore = follower.pathBuilder()
            .addPath(Path(BezierLine(blueTopRowEnd, blueScore)))
            .setLinearHeadingInterpolation(blueTopRowEnd.heading, blueScore.heading)
            .build()

        blueScoreToMiddle = follower.pathBuilder()
            .addPath(Path(BezierLine(blueScore, blueMiddleRowStart)))
            .setLinearHeadingInterpolation(blueScore.heading, blueMiddleRowStart.heading)
            .build()

        blueMiddleIntake = follower.pathBuilder()
            .addPath(Path(BezierLine(blueMiddleRowStart, blueMiddleRowEnd)))
            .setLinearHeadingInterpolation(blueMiddleRowStart.heading, blueMiddleRowEnd.heading)
            .build()

        blueMiddleToScore = follower.pathBuilder()
            .addPath(Path(BezierLine(blueMiddleRowEnd, blueScore)))
            .setLinearHeadingInterpolation(blueMiddleRowEnd.heading, blueScore.heading)
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
            0 -> { follower.followPath(blueStartToScore); outtake.launch(); limelight.useAprilTagPipeline(); actionTimer.resetTimer(); actionState = 0; setPathState(1) }
            1 -> if (!follower.isBusy) { follower.followPath(blueScoreToTop); setPathState(2) }
            2 -> if (!follower.isBusy) { intake.collect(); follower.followPath(blueTopIntake); setPathState(3) }
            3 -> if (!follower.isBusy) { intake.stop(); follower.followPath(blueTopToScore); outtake.launch(); limelight.useAprilTagPipeline(); actionTimer.resetTimer(); actionState = 0; setPathState(4) }
            4 -> if (!follower.isBusy) { follower.followPath(blueScoreToMiddle); setPathState(5) }
            5 -> if (!follower.isBusy) { intake.collect(); follower.followPath(blueMiddleIntake); setPathState(6) }
            6 -> if (!follower.isBusy) { intake.stop(); follower.followPath(blueMiddleToScore); outtake.launch(); limelight.useAprilTagPipeline(); actionTimer.resetTimer(); actionState = 0; setPathState(7) }
            7 -> if (!follower.isBusy) { follower.followPath(blueScoreToBottom); setPathState(8) }
            8 -> if (!follower.isBusy) { intake.collect(); follower.followPath(blueBottomIntake); setPathState(9) }
            9 -> if (!follower.isBusy) { intake.stop(); follower.followPath(blueBottomToScore); outtake.launch(); limelight.useAprilTagPipeline(); actionTimer.resetTimer(); actionState = 0; setPathState(10) }
            10 -> if (!follower.isBusy) { setPathState(11) }
            11 -> { }
        }
    }

    private fun shootingSequence() {
        if (pathState < 1 || pathState > 10) return
        when (actionState) {
            0 -> if (actionTimer.getElapsedTime() > 350) { transfer.transfer(); actionTimer.resetTimer(); actionState = 1 }
            1 -> if (actionTimer.getElapsedTime() > 600) { outtake.launch(); actionTimer.resetTimer(); actionState = 2 }
            2 -> if (actionTimer.getElapsedTime() > 300) { spindexer.switch(); actionTimer.resetTimer(); actionState = 3 }
            3 -> if (actionTimer.getElapsedTime() > 350) { transfer.transfer(); actionTimer.resetTimer(); actionState = 4 }
            4 -> if (actionTimer.getElapsedTime() > 600) { outtake.launch(); actionTimer.resetTimer(); actionState = 5 }
            5 -> if (actionTimer.getElapsedTime() > 300) { spindexer.switch(); actionTimer.resetTimer(); actionState = 6 }
            6 -> if (actionTimer.getElapsedTime() > 350) { transfer.transfer(); actionTimer.resetTimer(); actionState = 7 }
            7 -> if (actionTimer.getElapsedTime() > 600) { outtake.launch(); actionTimer.resetTimer(); actionState = 0 }
        }
    }

    private fun setPathState(state: Int) { pathState = state; pathTimer.resetTimer() }
}
