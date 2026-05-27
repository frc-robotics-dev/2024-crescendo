package frc.lib.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.path.RotationTarget;

public final class GeometricFollower {
    private static final double EPS = 1e-9;

    /** A point on the path with optional holonomic heading. */
    public static final class Sample {
        public final Translation2d pos;
        public final double s; // 0–1 progress
        public final Optional<Rotation2d> heading;

        public Sample(Translation2d pos, double s, Optional<Rotation2d> heading) {
            this.pos = pos;
            this.s = s;
            this.heading = heading;
        }
    }

    private final List<Sample> samples;
    private final PIDController headingController;
    private final double kForward;
    private final double kCross;

    private double progress = 0.0;

    public GeometricFollower(
        List<Sample> samples,
        PIDController headingController,
        double kForward,
        double kCross
    ) {
        if (samples.size() < 2) {
            throw new IllegalArgumentException("Need at least 2 samples");
        }
        this.samples = samples;
        this.headingController = headingController;
        this.kForward = kForward;
        this.kCross = kCross;
    }

    private static final class Closest {
        final Translation2d point;
        final int seg;
        final double s;

        Closest(Translation2d point, int seg, double s) {
            this.point = point;
            this.seg = seg;
            this.s = s;
        }
    }

    public ChassisSpeeds update(Pose2d robotPose, double speedMps) {
        Closest c = findClosest(robotPose.getTranslation());

        // Tangent direction
        Translation2d p0 = samples.get(c.seg).pos;
        Translation2d p1 = samples.get(c.seg + 1).pos;
        Translation2d seg = p1.minus(p0);
        Translation2d tangent =
            seg.getNorm() > EPS ? seg.div(seg.getNorm()) : new Translation2d(1, 0);

        // Cross-track correction
        Translation2d errVec = c.point.minus(robotPose.getTranslation());
        double e = errVec.getNorm();
        Translation2d normal = e > EPS ? errVec.div(e) : new Translation2d();

        // BLine-style combination
        Translation2d dir =
            tangent.times(kForward)
                .plus(normal.times(kCross * e));

        if (dir.getNorm() > EPS) {
            dir = dir.div(dir.getNorm());
        }

        Translation2d velField = dir.times(speedMps);

        // Heading interpolation
        Rotation2d desiredHeading = interpolateHeading(c.s, robotPose.getRotation());
        double omega = headingController.calculate(
            robotPose.getRotation().getRadians(),
            desiredHeading.getRadians()
        );

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            velField.getX(),
            velField.getY(),
            omega,
            robotPose.getRotation()
        );
    }

    private Closest findClosest(Translation2d robot) {
        double best = Double.POSITIVE_INFINITY;
        Translation2d bestP = samples.get(0).pos;
        int bestSeg = 0;
        double bestS = samples.get(0).s;

        for (int i = 0; i < samples.size() - 1; i++) {
            Sample a = samples.get(i);
            Sample b = samples.get(i + 1);

            Translation2d ab = b.pos.minus(a.pos);
            double ab2 = ab.getNorm() * ab.getNorm();

            double t;
            if (ab2 < EPS) {
                t = 0.0;
            } else {
                Translation2d ar = robot.minus(a.pos);
                t = (ar.getX() * ab.getX() + ar.getY() * ab.getY()) / ab2;
                t = Math.max(0.0, Math.min(1.0, t));
            }

            Translation2d proj = a.pos.plus(ab.times(t));
            double d = proj.getDistance(robot);

            if (d < best) {
                best = d;
                bestP = proj;
                bestSeg = i;
                bestS = a.s + (b.s - a.s) * t;
            }
        }

        progress = Math.max(progress, bestS);
        return new Closest(bestP, bestSeg, bestS);
    }

    private Rotation2d interpolateHeading(double s, Rotation2d fallback) {
        Optional<Rotation2d> prev = Optional.empty();
        Optional<Rotation2d> next = Optional.empty();
        double prevS = 0.0;
        double nextS = 1.0;

        for (Sample sm : samples) {
            if (sm.heading.isEmpty()) continue;

            if (sm.s <= s && sm.s >= prevS) {
                prev = sm.heading;
                prevS = sm.s;
            }
            if (sm.s >= s && sm.s <= nextS) {
                next = sm.heading;
                nextS = sm.s;
            }
        }

        if (prev.isEmpty() && next.isEmpty()) return fallback;
        if (next.isEmpty()) return prev.get();
        if (prev.isEmpty()) return next.get();

        double t = (s - prevS) / Math.max(EPS, (nextS - prevS));
        return prev.get().interpolate(next.get(), t);
    }

    public boolean isFinished() {
        return progress >= 0.999;
    }

    public static List<GeometricFollower.Sample> buildSamplesFromPathPlanner(
        List<Translation2d> positions,
        List<RotationTarget> rotationTargets
    ) {
        List<GeometricFollower.Sample> out = new ArrayList<>();

        if (positions.size() < 2) {
            throw new IllegalArgumentException("Need at least 2 points");
        }

        // Compute segment lengths
        double total = 0.0;
        List<Double> seg = new ArrayList<>();
        seg.add(0.0);

        for (int i = 1; i < positions.size(); i++) {
            double d = positions.get(i).getDistance(positions.get(i - 1));
            seg.add(d);
            total += d;
        }
        if (total < 1e-9) total = 1.0;

        // Build samples
        double accum = 0.0;
        for (int i = 0; i < positions.size(); i++) {
            accum += seg.get(i);
            double s = accum / total;

            Optional<Rotation2d> heading = Optional.empty();
            // YOU fill this in:
            // find rotationTarget whose position matches this sample’s s
            // or interpolate between rotationTargets

            out.add(new GeometricFollower.Sample(positions.get(i), s, heading));
        }

        return out;
    }
}