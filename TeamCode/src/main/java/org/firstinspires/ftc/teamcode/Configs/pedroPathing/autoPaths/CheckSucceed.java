package org.firstinspires.ftc.teamcode.Configs.pedroPathing.autoPaths;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

public class CheckSucceed {

    private final static PoseFactory poseFactory = PoseFactory.degrees();

    private final static Pose start = poseFactory.of(41.75, 8, 270);
    private final static Pose point1 = poseFactory.of(6.25, 18.2619, 270);
    private final static Pose point2Start = poseFactory.of(6.25, 18.2619, 270);
    private final static Pose point2 = poseFactory.of(6.25, 7, 270);
    private final static Pose point3 = poseFactory.of(23.6274, 15.2415, 430);
    private final static Pose point3Control1 = poseFactory.of(13.6657, 19.1955, 0);
    private final static Pose point3Segment1Heading = poseFactory.of(23.6274, 15.2415, 270);
    private final static Pose point3Segment2Start = poseFactory.of(23.6274, 15.2415, 270);
    private final static Pose point3Segment2End = poseFactory.of(23.6274, 15.2415, 70);
    private final static Pose point4 = poseFactory.of(44.3135, 123.8371, 49.2195);
    private final static Pose point4Control1 = poseFactory.of(34.75, 112.75, 0);
    private final static Pose point5 = poseFactory.of(45.1268, 130.4122, 82.9494);
    private final static Pose point6 = poseFactory.of(13.5384, 115.5533, 25.1919);

    public static Path path1() {
        return Paths.line(start, point1).linear(start, point1);
    }

    public static Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public static Path path3() {
        return Paths.curve(point2, point3Control1, point3).heading(Interpolator.piecewise().until(0.4681, Interpolator.constant(point3Segment1Heading)).until(1, Interpolator.linear(point3Segment2Start, point3Segment2End)));
    }

    public static Path path4() {
        return Paths.curve(point3, point4Control1, point4).tangent();
    }

    public static Path path5() {
        return Paths.line(point4, point5).tangent();
    }

    public static Path path6() {
        return Paths.line(point5, point6).reverseTangent();
    }
}
