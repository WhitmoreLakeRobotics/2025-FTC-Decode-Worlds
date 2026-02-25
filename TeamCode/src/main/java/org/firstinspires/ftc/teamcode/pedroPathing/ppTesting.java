package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Settings;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
@Autonomous(name = "ppTesting", group = "PP")
public class ppTesting extends OpMode{

    public static Follower follower;


    public static Pose startPose = new Pose(57, 9, Math.toRadians(90)); // Start Pose of our robot.
    public static Pose scorePose = new Pose(57, 15, Math.toRadians(114)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //private final Pose scorePose = new Pose(wallScoreX, wallScoreY, wallScoreH); // seeing if configurables work for this. Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePoseAP =new Pose(52,18,Math.toRadians(10));

    private PathChain scorePreload;
    private PathChain grabPickup1, grabPickup1a, grabPickup1b, grabPickup1c, scorePickup1, grabPickup2a,grabPickup2b, scorePickup2 ,goEndPose, goEndPose2, endPath;
    private PathChain cyclePickup1;



    cyclePickup1 = follower.pathBuilder()
            .addPath(new BezierLine(scorePose, pickup1aPose))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1aPose.getHeading())

            .addPath(new BezierLine(pickup1aPose, pickup1bPose))
            .setLinearHeadingInterpolation(pickup1aPose.getHeading(), pickup1bPose.getHeading())
            .addPath(new BezierCurve(pickup1bPose, scorePoseAP))

            .setLinearHeadingInterpolation(pickup1bPose.getHeading(), scorePose.getHeading())

            .build();

    @Override
    public void init() {


        follower =  CompBotConstants.createFollower(hardwareMap);
        buildPaths();
        PanelsConfigurables.INSTANCE.refreshClass(this);
        follower.setStartingPose(startPose);
        follower.update();





    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void loop() {

    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
