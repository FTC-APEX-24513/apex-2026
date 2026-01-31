package org.firstinspires.ftc.teamcode.mercurialftc.examples.autointoteleop

import android.provider.SyncStateContract
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.mercurialftc.examples.DemoSubsystem
import org.firstinspires.ftc.teamcode.opmodes.auto.PedroAutonomous.Paths
import org.mercurialftc.mercurialftc.scheduler.OpModeEX
import org.mercurialftc.mercurialftc.scheduler.Scheduler
import java.io.IOException

/**
 * see readme.md in this directory
 */
@Disabled // if you have the preselect option enabled, it will auto select teleop to run after this
@Autonomous(preselectTeleOp = "Teleop")
class PedroAutonomous : OpMode() {
    private var panelsTelemetry: TelemetryManager? = null // Panels Telemetry instance
    var follower: Follower? = null // Pedro Pathing follower instance
    private var pathState = 0 // Current autonomous path state (state machine)
    private var paths: Paths? = null // Paths defined in the Paths class

    public override fun init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry()

        follower = SyncStateContract.Constants.createFollower(hardwareMap)
        follower!!.setStartingPose(Pose(72.0, 8.0, Math.toRadians(90.0)))

        paths = Paths(follower!!) // Build paths

        panelsTelemetry.debug("Status", "Initialized")
        panelsTelemetry.update(telemetry)
    }
    class Auto : OpModeEX() {
        private var demoSubsystem: DemoSubsystem? = null

        public override fun registerSubsystems() {
            demoSubsystem = DemoSubsystem(this)
        }

        public override fun initEX() {
            Path1 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(124.000, 122.500),

                    Pose(84.000, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(36.0), Math.toRadians(45.0))

                .build()

            Path2 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(84.000, 84.000),

                    Pose(103.000, 60.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(0.0))

                .build()

            Path3 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(103.000, 60.000),

                    Pose(125.000, 60.000)
                )
            ).setTangentHeadingInterpolation()

                .build()

            Path4 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(125.000, 60.000),

                    Pose(84.000, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(45.0))

                .build()

            Path5 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(84.000, 84.000),

                    Pose(130.000, 63.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(30.0))

                .build()

            Path8 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(130.000, 63.000),

                    Pose(84.000, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(30.0), Math.toRadians(45.0))

                .build()

            Path9 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(84.000, 84.000),

                    Pose(104.000, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(0.0))

                .build()

            Path10 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(104.000, 84.000),

                    Pose(125.000, 84.000)
                )
            ).setTangentHeadingInterpolation()

                .build()

            Path11 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(125.000, 84.000),

                    Pose(84.000, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(45.0))

                .build()

            Path10 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(84.000, 84.000),

                    Pose(103.000, 36.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(0.0))

                .build()

            Path11 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(103.000, 36.000),

                    Pose(125.000, 36.000)
                )
            ).setTangentHeadingInterpolation()

                .build()

            Path12 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(125.000, 36.000),

                    Pose(84.000, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(45.0))

                .build()
        }
    }
}

}

public override fun init_loopEX() {
}

public override fun startEX() {
    // reset everything!

    // run your not asynchronous auto code here!
}

public override fun loopEX() {
    follower!!.update() // Update Pedro Pathing
    pathState = autonomousPathUpdate() // Update autonomous state machine

    // Log values to Panels and Driver Station
    panelsTelemetry.debug("Path State", pathState)
    panelsTelemetry.debug("X", follower!!.getPose().getX())
    panelsTelemetry.debug("Y", follower!!.getPose().getY())
    panelsTelemetry.debug("Heading", follower!!.getPose().getHeading())
    panelsTelemetry.update(telemetry)
}

public override fun stopEX() {
    // we do NOT want to refresh the scheduler when we swap to teleop
    Scheduler.getConfigOptionsManager()
        .updateValue(Scheduler.ConfigOptions.SCHEDULER_REFRESH_ENABLED.getOption(), false)


    try {
        Scheduler.getConfigOptionsManager().update() // actually updates the setting we changed
    } catch (e: IOException) {
        throw RuntimeException(e)
    }
    // add each subsystem to the stored subsystems arraylist in the scheduler
    // we made sure not to wipe the scheduler, so we can get them back!
    getScheduler().storeSubsystem(
        "my very own Demosubsystem",
        demoSubsystem
    ) // we give it a unique name, so we can have multiple of one subsystem
    //		getScheduler().storeSubsystem("demosubsystem2", demoSubsystem2); (this doesn't work bc i don't have more than one subsystem, but for demonstration's sake)
}
fun autonomousPathUpdate(): Int {
    // Event markers will automatically trigger at their positions
    // Make sure to register NamedCommands in your RobotContainer
    return pathState
}
}