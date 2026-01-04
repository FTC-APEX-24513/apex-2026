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

@Autonomous(name = "Example Auto", group = "Examples")
abstract class BlueTop : OpMode() {

    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer

    private var pathState = 0

    // Subsystems
    private lateinit var intake: IntakeSubsystem
    private lateinit var outtake: OuttakeSubsystem
    private lateinit var transfer: TransferSubsystem
    private lateinit var spindexer: SpindexerSubsystem
    private lateinit var limelight: LimelightSubsystem

    // Paths
    private lateinit var scorePreload: Path
    private lateinit var blueScoreToTopStart: PathChain
    private lateinit var blueTopStartToTopEnd: PathChain
    private lateinit var blueTopEndToScore: PathChain
    private lateinit var blueScoreToMiddleStart: PathChain
    private lateinit var blueMiddleStartToMiddleEnd: PathChain
    private lateinit var blueMiddleEndToScore: PathChain
    private lateinit var blueScoreToBottomStart: PathChain
    private lateinit var blueBottomStartToBottomEnd: PathChain
    private lateinit var blueBottomEndToScore: PathChain

    // Poses
    private val startPoseBlueTop = Pose(56.0, 88.0, Math.toRadians(180.0))
    private val blueTopRowStart = Pose(40.0, 84.0, Math.toRadians(180.0))
    private val blueTopRowEnd = Pose(20.0, 84.0, Math.toRadians(180.0))
    private val blueMiddleRowStart = Pose(40.0, 60.0, Math.toRadians(180.0))
    private val blueMiddleRowEnd = Pose(20.0, 60.0, Math.toRadians(180.0))
    private val blueBottomRowStart = Pose(40.0, 36.0, Math.toRadians(180.0))
    private val blueBottomRowEnd = Pose(20.0, 36.0, Math.toRadians(180.0))
    private val blueScore = Pose(48.0, 48.0, Math.toRadians(180.0)) // tune after testing

    override fun init() {
        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()

        follower = Constants.createFollower(hardwareMap)
        buildPaths()
        follower.setStartingPose(startPoseBlueTop)

        // Initialize subsystems
        intake = IntakeSubsystem(hardwareMap)
        outtake = OuttakeSubsystem(hardwareMap)
        transfer = TransferSubsystem(hardwareMap)
        spindexer = SpindexerSubsystem(hardwareMap)
        limelight = LimelightSubsystem(hardwareMap)
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
        telemetry.addData("X", follower.pose.x)
        telemetry.addData("Y", follower.pose.y)
        telemetry.addData("Heading", follower.pose.heading)
        telemetry.update()
    }

    private fun buildPaths() {
        scorePreload = Path(BezierLine(startPoseBlueTop, blueScore))
        scorePreload.setLinearHeadingInterpolation(startPoseBlueTop.heading, blueScore.heading)

        blueScoreToTopStart = follower.pathBuilder()
            .addPath(BezierLine(blueScore, blueTopRowStart))
            .setLinearHeadingInterpolation(blueScore.heading, blueTopRowStart.heading)
            .build()

        blueTopStartToTopEnd = follower.pathBuilder()
            .addPath(BezierLine(blueTopRowStart, blueTopRowEnd))
            .setLinearHeadingInterpolation(blueTopRowStart.heading, blueTopRowEnd.heading)
            .build()

        blueTopEndToScore = follower.pathBuilder()
            .addPath(BezierLine(blueTopRowEnd, blueScore))
            .setLinearHeadingInterpolation(blueTopRowEnd.heading, blueScore.heading)
            .build()

        blueScoreToMiddleStart = follower.pathBuilder()
            .addPath(BezierLine(blueScore, blueMiddleRowStart))
            .setLinearHeadingInterpolation(blueScore.heading, blueMiddleRowStart.heading)
            .build()

        blueMiddleStartToMiddleEnd = follower.pathBuilder()
            .addPath(BezierLine(blueMiddleRowStart, blueMiddleRowEnd))
            .setLinearHeadingInterpolation(blueMiddleRowStart.heading, blueMiddleRowEnd.heading)
            .build()

        blueMiddleEndToScore = follower.pathBuilder()
            .addPath(BezierLine(blueMiddleRowEnd, blueScore))
            .setLinearHeadingInterpolation(blueMiddleRowEnd.heading, blueScore.heading)
            .build()

        blueScoreToBottomStart = follower.pathBuilder()
            .addPath(BezierLine(blueScore, blueBottomRowStart))
            .setLinearHeadingInterpolation(blueScore.heading, blueBottomRowStart.heading)
            .build()

        blueBottomStartToBottomEnd = follower.pathBuilder()
            .addPath(BezierLine(blueBottomRowStart, blueBottomRowEnd))
            .setLinearHeadingInterpolation(blueBottomRowStart.heading, blueBottomRowEnd.heading)
            .build()

        blueBottomEndToScore = follower.pathBuilder()
            .addPath(BezierLine(blueBottomRowEnd, blueScore))
            .setLinearHeadingInterpolation(blueBottomRowEnd.heading, blueScore.heading)
            .build()
    }

    private fun autonomousPathUpdate() {
        when (pathState) {
            0 -> { // Start → first score
                follower.followPath(scorePreload)
                outtake.launch()
                limelight.useAprilTagPipeline()
                actionTimer.resetTimer()
                pathState = 1
            }
            1 -> if (!follower.isBusy) {
                follower.followPath(blueScoreToTopStart)
                pathState = 2
            }
            2 -> if (!follower.isBusy) {
                intake.collect()
                follower.followPath(blueTopStartToTopEnd)
                pathState = 3
            }
            3 -> if (!follower.isBusy) {
                intake.stop()
                follower.followPath(blueTopEndToScore)
                limelight.useAprilTagPipeline()
                outtake.launch()
                actionTimer.resetTimer()
                pathState = 4
            }
            4 -> if (!follower.isBusy) {
                follower.followPath(blueScoreToMiddleStart)
                pathState = 5
            }
            5 -> if (!follower.isBusy) {
                intake.collect()
                follower.followPath(blueMiddleStartToMiddleEnd)
                pathState = 6
            }
            6 -> if (!follower.isBusy) {
                intake.stop()
                follower.followPath(blueMiddleEndToScore)
                limelight.useAprilTagPipeline()
                outtake.launch()
                actionTimer.resetTimer()
                pathState = 7
            }
            7 -> if (!follower.isBusy) {
                follower.followPath(blueScoreToBottomStart)
                pathState = 8
            }
            8 -> if (!follower.isBusy) {
                intake.collect()
                follower.followPath(blueBottomStartToBottomEnd)
                pathState = 9
            }
            9 -> if (!follower.isBusy) {
                intake.stop()
                follower.followPath(blueBottomEndToScore)
                limelight.useAprilTagPipeline()
                outtake.launch()
                actionTimer.resetTimer()
                pathState = 10
            }
            10 -> if (!follower.isBusy) {
                pathState = 11
            }
            11 -> {
                // Auto finished
            }
        }
    }

    private fun shootingSequence() {
        if (pathState < 1 || pathState > 10) return

        // Spindexer uses switch() instead of advance()
        when (actionTimer.getElapsedTime().toInt()) {
            in 0..350 -> transfer.transfer()
            in 351..950 -> outtake.launch()
            in 951..1250 -> spindexer.switch()
            in 1251..1600 -> transfer.transfer()
            in 1601..2200 -> outtake.launch()
            in 2201..2500 -> spindexer.switch()
            in 2501..2850 -> transfer.transfer()
            in 2851..3450 -> outtake.launch()
        }
    }

    private fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }
}
