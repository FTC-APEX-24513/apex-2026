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
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem

@Autonomous(name = "redTop3", group = "Examples")
abstract class redTop3: OpMode() {

    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer

    private var pathState = 0

    // Subsystems
    private lateinit var intake: IntakeSubsystem
    private lateinit var outtake: OuttakeSubsystem
    private lateinit var transfer: TransferSubsystem
    private lateinit var spindexer: SpindexerSubsystem
    private lateinit var limelight: LimelightSubsystem

    // Paths
    private lateinit var scorePreload: Path
    private lateinit var redScoreToTopStart: PathChain
    private lateinit var redTopStartToTopEnd: PathChain
    private lateinit var redTopEndToScore: PathChain
    private lateinit var redScoreToMiddleStart: PathChain
    private lateinit var redMiddleStartToMiddleEnd: PathChain
    private lateinit var redMiddleEndToScore: PathChain
    private lateinit var redScoreToBottomStart: PathChain
    private lateinit var redBottomStartToBottomEnd: PathChain
    private lateinit var redBottomEndToScore: PathChain

    // Poses
    private val startPoseredTop =
        Pose(88.0, 88.0, Math.toRadians(180.0)) //Aligned with top left of robot on Y4
    private val startPoseredBottom =
        Pose(88.0, 16.0, Math.toRadians(180.0)) //Aligned with left right of robot on Y1
    private val redTopRowStart = Pose(104.0, 84.0, Math.toRadians(90.0))
    private val redTopRowEnd = Pose(124.0, 84.0, Math.toRadians(90.0))
    private val redMiddleRowStart = Pose(104.0, 60.0, Math.toRadians(180.0))
    private val redMiddleRowEnd = Pose(124.0, 60.0, Math.toRadians(180.0))
    private val redBottomRowStart = Pose(104.0, 36.0, Math.toRadians(180.0))
    private val redBottomRowEnd = Pose(124.0, 36.0, Math.toRadians(180.0))

    override fun init() {
        pathTimer = Timer()
        follower = Constants.createFollower(hardwareMap)
        buildPaths()
        follower.setStartingPose(startPoseredTop)

        intake = IntakeSubsystem(hardwareMap)
        outtake = OuttakeSubsystem(hardwareMap)
        transfer = TransferSubsystem(hardwareMap)
        spindexer = SpindexerSubsystem(hardwareMap)
        limelight = LimelightSubsystem(hardwareMap)
    }

    override fun start() {
        setPathState(0)
    }

    override fun loop() {
        follower.update()
        autonomousPathUpdate()
    }

    private fun buildPaths() {
        scorePreload = Path(BezierLine(startPoseredTop, redScore))
        scorePreload.setLinearHeadingInterpolation(startPoseredTop.heading, redScore.heading)

        redScoreToTopStart = follower.pathBuilder()
            .addPath(BezierLine(redScore, redTopRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redTopRowStart.heading)
            .build()

        redTopStartToTopEnd = follower.pathBuilder()
            .addPath(BezierLine(redTopRowStart, redTopRowEnd))
            .setLinearHeadingInterpolation(redTopRowStart.heading, redTopRowEnd.heading)
            .build()

        redTopEndToScore = follower.pathBuilder()
            .addPath(BezierLine(redTopRowEnd, redScore))
            .setLinearHeadingInterpolation(redTopRowEnd.heading, redScore.heading)
            .build()

        redScoreToMiddleStart = follower.pathBuilder()
            .addPath(BezierLine(redScore, redMiddleRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redMiddleRowStart.heading)
            .build()

        redMiddleStartToMiddleEnd = follower.pathBuilder()
            .addPath(BezierLine(redMiddleRowStart, redMiddleRowEnd))
            .setLinearHeadingInterpolation(redMiddleRowStart.heading, redMiddleRowEnd.heading)
            .build()

        redMiddleEndToScore = follower.pathBuilder()
            .addPath(BezierLine(redMiddleRowEnd, redScore))
            .setLinearHeadingInterpolation(redMiddleRowEnd.heading, redScore.heading)
            .build()

        redScoreToBottomStart = follower.pathBuilder()
            .addPath(BezierLine(redScore, redBottomRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redBottomRowStart.heading)
            .build()

        redBottomStartToBottomEnd = follower.pathBuilder()
            .addPath(BezierLine(redBottomRowStart, redBottomRowEnd))
            .setLinearHeadingInterpolation(redBottomRowStart.heading, redBottomRowEnd.heading)
            .build()

        redBottomEndToScore = follower.pathBuilder()
            .addPath(BezierLine(redBottomRowEnd, redScore))
            .setLinearHeadingInterpolation(redBottomRowEnd.heading, redScore.heading)
            .build()
    }

    private fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                follower.followPath(scorePreload)
                outtake.launch()
                pathState = 1
            }
            1 -> if (!follower.isBusy) {
                follower.followPath(redScoreToTopStart)
                pathState = 2
            }
            2 -> if (!follower.isBusy) {
                intake.collect()
                follower.followPath(redTopStartToTopEnd)
                pathState = 3
            }
            3 -> if (!follower.isBusy) {
                intake.stop()
                follower.followPath(redTopEndToScore)
                outtake.launch()
                pathState = 4
            }
            4 -> if (!follower.isBusy) {
                follower.followPath(redScoreToMiddleStart)
                pathState = 5
            }
            5 -> if (!follower.isBusy) {
                intake.collect()
                follower.followPath(redMiddleStartToMiddleEnd)
                pathState = 6
            }
            6 -> if (!follower.isBusy) {
                intake.stop()
                follower.followPath(redMiddleEndToScore)
                outtake.launch()
                pathState = 7
            }
            7 -> if (!follower.isBusy) {
                follower.followPath(redScoreToBottomStart)
                pathState = 8
            }
            8 -> if (!follower.isBusy) {
                intake.collect()
                follower.followPath(redBottomStartToBottomEnd)
                pathState = 9
            }
            9 -> if (!follower.isBusy) {
                intake.stop()
                follower.followPath(redBottomEndToScore)
                outtake.launch()
                pathState = 10
            }
            10 -> if (!follower.isBusy) {
                pathState = 11
            }
            11 -> { /* Auto finished */ }
        }
    }

    private fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }
}
