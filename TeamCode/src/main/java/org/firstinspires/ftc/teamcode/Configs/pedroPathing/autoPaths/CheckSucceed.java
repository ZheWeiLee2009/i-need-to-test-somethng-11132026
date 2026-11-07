
package org.firstinspires.ftc.teamcode.Configs.pedroPathing.autoPaths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

public class CheckSucceed {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    private static final Pose start = poseFactory.of(6, 6.5, 90);
    private static final Pose alignmenttoPATH1Start = poseFactory.of(55.4964, 8.3783, 270);
    private static final Pose point2Start = poseFactory.of(55.4964, 8.3783, 270);
    private static final Pose point2 = poseFactory.of(7, 18.2619, 270);
    private static final Pose point3Start = poseFactory.of(7, 18.2619, 270);
    private static final Pose point3 = poseFactory.of(6.8739, 6.7478, 270);
    private static final Pose point4 = poseFactory.of(36.2388, 21.6734, 430);
    private static final Pose point4Control1 = poseFactory.of(12.0262, 25.5012, 0);
    private static final Pose point4Segment1Heading = poseFactory.of(36.2388, 21.6734, 270);
    private static final Pose point4Segment2Start = poseFactory.of(36.2388, 21.6734, 270);
    private static final Pose point4Segment2End = poseFactory.of(36.2388, 21.6734, 70);
    private static final Pose point5Start = poseFactory.of(36.2388, 21.6734, 90);
    private static final Pose point5 = poseFactory.of(47.8447, 116.7747, 75);
    private static final Pose point5Control1 = poseFactory.of(25.1653, 84.3743, 0);
    private static final Pose point6 = poseFactory.of(48.0274, 124.64, 88.8198);
    private static final Pose point7Start = poseFactory.of(48.0274, 124.64, 90);
    private static final Pose point7 = poseFactory.of(13.2862, 116.0578, 0);

    public static Pose startPose() {
        return start;
    }

    public static Path alignmenttoPATH1Start() {
        return Paths.line(start, alignmenttoPATH1Start).linear(start, alignmenttoPATH1Start);
    }

    public static Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public static Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public static Path path4() {
        return Paths.curve(point3, point4Control1, point4).heading(Interpolator.piecewise().until(0.4681, Interpolator.constant(point4Segment1Heading)).until(1, Interpolator.linear(point4Segment2Start, point4Segment2End)));
    }

    public static Path path5() {
        return Paths.curve(point5Start, point5Control1, point5).linear(point5Start, point5);
    }

    public static Path path6() {
        return Paths.line(point5, point6).tangent();
    }

    public static Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }
}
