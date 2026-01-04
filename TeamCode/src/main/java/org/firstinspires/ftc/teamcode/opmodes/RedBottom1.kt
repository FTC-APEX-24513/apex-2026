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
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem

@Autonomous(name = "redBottom_1_Kotlin", group = "Examples")
class RedBottom1OpMode : OpMode() {

    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer
    private var pathState = 0

    // Poses for Red Bottom 1
    private val startPose = Pose(88.0, 16.0, Math.toRadians(180.0))
    private val redBottomRowStart = Pose(104.0, 36.0, Math.toRadians(180.0))
    private val redBottomRowEnd = Pose(124.0, 36.0, Math.toRadians(180.0))
    private val redScore = Pose()

    // Paths
    private lateinit var scorePreload: Path
    private lateinit var redScoreToBottom: PathChain
    private lateinit var redBottomIntake: PathChain
    private lateinit var redBottomToScore: PathChain

    // Subsystems
    private lateinit var intake: IntakeSubsystem
    private lateinit var outtake: OuttakeSubsystem
    private lateinit var limelight: LimelightSubsystem
    private lateinit var transfer: TransferSubsystem
    private lateinit var spindexer: SpindexerSubsystem

    override fun init() {
        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer.resetTimer()

        follower = Constants.createFollower(hardwareMap)
        buildPaths()
        follower.setStartingPose(startPose)

        intake = IntakeSubsystem(hardwareMap)
        outtake = OuttakeSubsystem(hardwareMap)
        limelight = LimelightSubsystem()
        limelight.init(hardwareMap, telemetry)
        transfer = TransferSubsystem(hardwareMap)
        spindexer = SpindexerSubsystem(hardwareMap)
    }

    private fun buildPaths() {
        // Straight to score
        scorePreload = Path(BezierLine(startPose, redScore)).apply {
            setLinearHeadingInterpolation(startPose.heading, redScore.heading)
        }

        // Score → bottom row start
        redScoreToBottom = follower.pathBuilder()
            .addPath(BezierLine(redScore, redBottomRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redBottomRowStart.heading)
            .build()

        // Bottom row start → bottom row end
        redBottomIntake = follower.pathBuilder()
            .addPath(BezierLine(redBottomRowStart, redBottomRowEnd))
            .setLinearHeadingInterpolation(redBottomRowStart.heading, redBottomRowEnd.heading)
            .build()

        // Bottom row end → score
        redBottomToScore = follower.pathBuilder()
            .addPath(BezierLine(redBottomRowEnd, redScore))
            .setLinearHeadingInterpolation(redBottomRowEnd.heading, redScore.heading)
            .build()
    }

    override fun start() {
        opmodeTimer.resetTimer()
        pathState = 0
    }

    override fun loop() {
        follower.update()
        autonomousPathUpdate()

        telemetry.addData("path state", pathState)
        telemetry.addData("x", follower.pose.x)
        telemetry.addData("y", follower.pose.y)
        telemetry.addData("heading", follower.pose.heading)
        telemetry.update()
    }

    private fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                follower.followPath(scorePreload)
                outtake.outtake() // Shoot preload
                pathState = 1
            }

            1 -> {
                if (!follower.isBusy) {
                    follower.followPath(redScoreToBottom)
                    pathState = 2
                }
            }

            2 -> {
                if (!follower.isBusy) {
                    intake.intake()
                    follower.followPath(redBottomIntake)
                    pathState = 3
                }
            }

            3 -> {
                if (!follower.isBusy) {
                    intake.stop()
                    follower.followPath(redBottomToScore)
                    limelight.update()
                    limelight.localize()
                    outtake.outtake() // Shoot collected bottom-row freight
                    pathState = 4
                }
            }

            4 -> {
                if (!follower.isBusy) {
                    pathState = 5 // End routine
                }
            }

            5 -> {
                // Routine finished
            }
        }
    }
}
