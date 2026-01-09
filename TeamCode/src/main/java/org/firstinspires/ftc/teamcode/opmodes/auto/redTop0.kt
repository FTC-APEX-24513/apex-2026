package org.firstinspires.ftc.teamcode.opmodes.auto

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

@Suppress("UNUSED")
val redTop0 = Mercurial.autonomous {
    // subsystems
    val outtake = OuttakeSubsystem(hardwareMap)
    val limelight = LimelightSubsystem()
    limelight.init(hardwareMap, telemetry)

    // pathing
    val follower: Follower = Constants.createFollower(hardwareMap)
    val pathTimer = Timer()
    var pathState = 0

    val startPoseRedTop = Pose(88.0, 88.0, Math.toRadians(180.0))
    val redScore = Pose() // as in original

    lateinit var redStartToScore: PathChain

    fun buildPaths() {
        redStartToScore = follower.pathBuilder()
            .addPath(BezierLine(startPoseRedTop, redScore))
            .setLinearHeadingInterpolation(startPoseRedTop.heading, redScore.heading)
            .build()
    }

    // initialize
    buildPaths()
    follower.setStartingPose(startPoseRedTop)

    // main autonomous sequence
    schedule(
        sequence(
            exec { pathState = 0 },
            loop(
                exec {
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
            )
        )
    )

    dropToScheduler()
}
