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
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem

@Autonomous(name = "redBottom_0_Kotlin", group = "Examples")
class RedBottom0OpMode : OpMode() {

    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer
    private var pathState = 0

    // Poses
    private val startPose = Pose(88.0, 16.0, Math.toRadians(180.0))
    private val redScore = Pose()

    // Path
    private lateinit var redStartToScore: PathChain

    // Subsystems
    private lateinit var outtake: OuttakeSubsystem
    private lateinit var limelight: LimelightSubsystem

    override fun init() {
        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer.resetTimer()

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
                // Drive straight to score
                follower.followPath(redStartToScore)
                outtake.outtake() // Shoot preload
                pathState = 1
            }

            1 -> {
                if (!follower.isBusy) {
                    pathState = 2 // End of routine
                }
            }

            2 -> {
                // Finished
            }
        }
    }
}
