package org.firstinspires.ftc.teamcode.opmodes.auto

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem

@Autonomous(name = "redBottom_0_Kotlin", group = "Examples")
class RedBottom0OpMode : OpMode() {

    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private var pathState = 0

    private val startPose = Pose(88.0, 16.0, Math.toRadians(180.0))
    private val redScore = Pose() // define scoring position

    private lateinit var redStartToScore: PathChain

    private lateinit var outtake: OuttakeSubsystem
    private lateinit var limelight: LimelightSubsystem

    override fun init() {
        pathTimer = Timer()
        follower = Constants.createFollower(hardwareMap)
        buildPaths()
        follower.setStartingPose(startPose)

        outtake = OuttakeSubsystem(hardwareMap)
        limelight = LimelightSubsystem()
        limelight.init(hardwareMap, telemetry)
    }

    private fun buildPaths() {
        redStartToScore = follower.pathBuilder()
            .addPath(BezierLine(startPose, redScore))
            .setLinearHeadingInterpolation(startPose.heading, redScore.heading)
            .build()
    }

    override fun start() {
        pathState = 0
    }

    override fun loop() {
        follower.update()
        when (pathState) {
            0 -> {
                follower.followPath(redStartToScore)
                outtake.outtake()
                pathState = 1
            }
            1 -> if (!follower.isBusy) pathState = 2
            2 -> { /* Finished */ }
        }
    }
}
