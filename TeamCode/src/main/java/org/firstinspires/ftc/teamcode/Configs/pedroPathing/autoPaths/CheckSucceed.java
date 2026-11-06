
package org.firstinspires.ftc.teamcode.Configs.pedroPathing.autoPaths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

public class CheckSucceed {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    // Starting position of the exported Pedro paths.
    private static final Pose start = poseFactory.of(41.75, 8, 270);

    private static final Pose point2Start = poseFactory.of(41.75, 8, 270);
    private static final Pose point2 = poseFactory.of(6.25, 18.2619477165, 270);

    private static final Pose point3Start = poseFactory.of(6.25, 18.2619477165, 270);
    private static final Pose point3 = poseFactory.of(6.25, 7, 270);

    private static final Pose point4 = poseFactory.of(23.6274394028, 15.2415335216, 75);
    private static final Pose point4Control1 = poseFactory.of(13.6656611242, 19.1955430017, 0);
    private static final Pose point4Segment1Heading = poseFactory.of(23.6274394028, 15.2415335216, 270);
    private static final Pose point4Segment2Start = poseFactory.of(23.6274394028, 15.2415335216, 270);
    private static final Pose point4Segment2End = poseFactory.of(23.6274394028, 15.2415335216, 70);

    private static final Pose point5 = poseFactory.of(44.3135460946, 123.8371099826, 90);
    private static final Pose point5Control1 = poseFactory.of(34.75, 112.75, 0);

    private static final Pose point6 = poseFactory.of(45.1267586225, 130.4121858635, 90);
    private static final Pose point7 = poseFactory.of(54.24, 79.7, 90);

    public static Pose startPose() {
        return start;
    }

    // PATH 2
    public static Path path2() {
        return Paths.line(point2Start, point2)
                .linear(point2Start, point2);
    }

    // PATH 3
    public static Path path3() {
        return Paths.line(point3Start, point3)
                .linear(point3Start, point3);
    }

    // PATH 4
    public static Path path4() {
        return Paths.curve(point3, point4Control1, point4)
                .heading(
                        Interpolator.piecewise()
                                .until(0.4680851064,
                                        Interpolator.constant(point4Segment1Heading))
                                .until(1,
                                        Interpolator.linear(point4Segment2Start, point4Segment2End))
                );
    }

    // PATH 5
    public static Path path5() {
        return Paths.curve(point4, point5Control1, point5)
                .tangent();
    }

    // PATH 6
    public static Path path6() {
        return Paths.line(point5, point6)
                .tangent();
    }

    // PATH 7
    public static Path path7() {
        return Paths.line(point6, point7)
                .reverseTangent();
    }
}
