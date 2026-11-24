#pragma once

#include "PoseMath.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <vector>

namespace GModPose
{
    // The view chooses horizontal directions. Vertical travel always follows world up.
    inline Vec3 PoseTravel(Vec3 look, Vec3 cameraRight, int forward, int across, int vertical, double dt)
    {
        const Vec3 up{0, 0, 1};
        const auto heading = Unit(Vec3{look.x, look.y, 0}).value_or(
            Unit(Cross(up, cameraRight)).value_or(Vec3{0, 1, 0}));
        const auto right = Cross(heading, up);
        const auto horizontal = heading * forward + right * across;
        const auto direction = Length(horizontal) > 1 ? *Unit(horizontal) : horizontal;
        return (direction + up * vertical) * (90 * dt);
    }

    struct LimbHinge
    {
        std::array<Vec3, 2> localNormals;
        double minBendAlignment, maxRoll;
        bool leg;
    };

    inline std::optional<LimbHinge> CaptureLimbHinge(const std::array<Transform, 3>& world, Vec3 straightBend, bool leg)
    {
        const auto upper = Unit(world[1].position - world[0].position);
        const auto lower = Unit(world[2].position - world[1].position);
        if (!upper || !lower) return std::nullopt;
        auto normal = Cross(*upper, *lower);
        if (Length(normal) < .035) {
            // A nearly straight limb has no stable measured plane.
            normal = Cross(straightBend, *upper);
            if (Length(normal) < .035) {
                const Vec3 across = std::abs(upper->z) < .9 ? Vec3{0, 0, 1} : Vec3{0, 1, 0};
                normal = Cross(across, *upper);
            }
        }
        const auto plane = Unit(normal);
        if (!plane) return std::nullopt;
        LimbHinge hinge{{}, leg ? .4 : .26, (leg ? 25 : 40) * std::numbers::pi / 180, leg};
        for (std::size_t i = 0; i < hinge.localNormals.size(); ++i) {
            const auto q = world[i].rotation;
            const auto local = Rotate({-q.x, -q.y, -q.z, q.w}, *plane);
            if (!local) return std::nullopt;
            hinge.localNormals[i] = *local;
        }
        return hinge;
    }

    struct LimbPose
    {
        std::array<Transform, 3> world;
        Quaternion parentRotation;
        // Actual node path from proximal joint to endpoint, including intermediate twist bones.
        std::vector<Quaternion> localRotations;
        std::size_t middle{};
        std::optional<LimbHinge> hinge{};
    };

    struct LimbEdit
    {
        std::array<Quaternion, 3> localRotations;
        Vec3 joint, tip, bend;
        Quaternion endRotation;
        bool limited{};
        Quaternion lowerRotation;
    };

    inline std::optional<LimbHinge> HingeAtGrip(const std::array<Transform, 3>& world, std::optional<LimbHinge> hinge)
    {
        if (!hinge || hinge->leg) return hinge;
        const auto upper = Unit(world[1].position - world[0].position);
        const auto lower = Unit(world[2].position - world[1].position);
        // A bent, authored arm defines its own roll on each side of the elbow.
        // Near extension the original hinge is a more stable reference.
        if (!upper || !lower || Length(Cross(*upper, *lower)) < .05) return hinge;
        return CaptureLimbHinge(world, {}, false);
    }

    struct LimbRotation
    {
        std::size_t joint;
        Quaternion world;
        double maxBendStep{};
    };

    struct TorsoClearance
    {
        Vec3 hips, chest, right, forward;
        double halfWidth, halfDepth;
        struct Head { Vec3 center; double radius; };
        std::optional<Head> head{};

        double Penetration(Vec3 point) const
        {
            const auto shaft = chest - hips;
            const auto along = std::clamp(Dot(point - hips, shaft) / Dot(shaft, shaft), 0.0, 1.0);
            const auto offset = point - (hips + shaft * along);
            const auto side = Dot(offset, right) / halfWidth;
            const auto depth = Dot(offset, forward) / halfDepth;
            const auto end = Dot(offset, shaft) / (Length(shaft) * halfDepth);
            return std::max(0.0, 1.0 - std::sqrt(side * side + depth * depth + end * end));
        }

        double LimbOverlap(Vec3 root, Vec3 joint, Vec3 tip) const
        {
            const auto upper = joint - root, lower = tip - joint;
            double total = 0;
            for (const auto point : {root + upper * .64, root + upper * .84, joint,
                 joint + lower * .22, joint + lower * .47, joint + lower * .72}) {
                const auto inside = Penetration(point);
                total += inside * inside;
            }
            const auto elbow = Penetration(joint);
            total += elbow * elbow;
            if (head) {
                const auto headOverlap = [&](Vec3 start, Vec3 shaft) {
                    const auto along = std::clamp(Dot(head->center - start, shaft) /
                        std::max(Dot(shaft, shaft), 1e-12), 0., 1.);
                    const auto depth = std::max(0., 1 - Length(start + shaft * along - head->center) / head->radius);
                    return depth * depth;
                };
                // Leave room for the wrist to touch the head when the forearm is clear.
                total += headOverlap(root, upper) + headOverlap(joint, lower * .85);
            }
            return total;
        }
    };

    inline std::optional<TorsoClearance> MakeTorsoClearance(Vec3 hips, Vec3 chest, Vec3 leftShoulder, Vec3 rightShoulder)
    {
        const auto up = Unit(chest - hips);
        if (!up) return std::nullopt;
        const auto span = rightShoulder - leftShoulder;
        const auto right = Unit(span - *up * Dot(span, *up));
        if (!right) return std::nullopt;
        const auto width = Dot(span, *right);
        if (width < .01) return std::nullopt;
        return TorsoClearance{hips, chest, *right, Cross(*up, *right), width * .42, width * .29};
    }

    inline double RollSine(Quaternion turn, Vec3 axis)
    {
        return 2 * turn.w * Dot(axis, {turn.x, turn.y, turn.z});
    }

    struct TwistPhase
    {
        double angle{};
        double origin{};

        static bool SameAngle(double left, double right) { return std::abs(left - right) <= 1e-6; }

        static double Deformation(double phase)
        {
            constexpr auto circle = 2 * std::numbers::pi;
            const auto magnitude = std::fmod(std::abs(phase), circle);
            if (magnitude <= std::numbers::pi) return std::copysign(magnitude, phase);
            // Keep the roll through the half-turn and return smoothly to the
            // captured deformation when the user finishes a complete turn.
            const auto finish = (circle - magnitude) / std::numbers::pi;
            return std::copysign(std::numbers::pi * finish * finish * (3 - 2 * finish), phase);
        }

        double Update(Quaternion turn, Vec3 axis)
        {
            const auto along = Dot(axis, {turn.x, turn.y, turn.z});
            // A half-turn perpendicular to the shaft has no defined twist.
            if (std::hypot(turn.w, along) > 1e-4) {
                const auto next = origin + 2 * std::atan2(along, turn.w);
                angle += std::remainder(next - angle, 2 * std::numbers::pi);
            }
            return Deformation(angle) - Deformation(origin);
        }
    };

    inline Quaternion DistributeLimbTwist(Quaternion captured, Vec3 axis, double proximal, double distal, double fraction)
    {
        return Multiply(AxisAngle(axis, std::lerp(-proximal, distal, fraction)), captured);
    }

    struct SegmentTwistFrame
    {
        Quaternion proximalRotation, distalRotation;
        Vec3 axis;
        TwistPhase proximal, distal;

        bool Begin(Quaternion joint, Transform endpoint)
        {
            const auto shaft = Unit(endpoint.position);
            if (!shaft) return false;
            axis = *shaft;
            proximalRotation = joint;
            distalRotation = endpoint.rotation;
            proximal.origin = proximal.angle;
            distal.origin = distal.angle;
            return true;
        }

        std::array<double, 2> Measure(Quaternion joint, Quaternion endpoint)
        {
            const auto p = proximalRotation, d = distalRotation;
            return {proximal.Update(Multiply({-p.x, -p.y, -p.z, p.w}, joint), axis),
                distal.Update(Multiply(endpoint, {-d.x, -d.y, -d.z, d.w}), axis)};
        }
    };

    inline Quaternion FollowLimbPlane(Quaternion swing, Quaternion rotation, Vec3 localNormal,
                                     Vec3 segment, Vec3 planeNormal)
    {
        const auto axis = Unit(segment);
        const auto swung = Rotate(Multiply(swing, rotation), localNormal);
        if (!axis || !swung) return swing;
        const auto tangent = Unit(*swung - *axis * Dot(*swung, *axis));
        if (!tangent) return swing;
        const auto correction = std::atan2(Dot(Cross(*tangent, planeNormal), *axis), Dot(*tangent, planeNormal));
        return Multiply(AxisAngle(*axis, correction), swing);
    }

    struct ArmDrag
    {
        std::array<Transform, 3> previous;
        double travel;
    };

    inline std::optional<Vec3> BendOnPlane(Vec3 bend, Vec3 axis)
    {
        return Unit(bend - axis * Dot(bend, axis));
    }

    inline std::optional<Vec3> CarryArmBend(const ArmDrag& drag, Vec3 axis, std::optional<Vec3> previousBend)
    {
        const auto& [shoulder, elbow, hand] = drag.previous;
        const auto previousAxis = Unit(hand.position - shoulder.position);
        if (!previousAxis) return std::nullopt;
        const auto bend = previousBend ? BendOnPlane(*previousBend, *previousAxis) :
            BendOnPlane(elbow.position - shoulder.position, *previousAxis);
        if (!bend) return std::nullopt;
        // The previous bend is the turn axis when the hand crosses behind the shoulder.
        if (Dot(*previousAxis, axis) < -.98) return BendOnPlane(*bend, axis);
        const auto carry = FromTo(*previousAxis, axis);
        return carry ? Rotate(*carry, *bend) : std::nullopt;
    }

    inline Quaternion CarryArmRoll(Quaternion previous, Vec3 previousSegment, Vec3 segment,
                                   Quaternion desired, double maxStep, double follow = 1.0)
    {
        const auto axis = Unit(segment);
        const auto swing = FromTo(previousSegment, segment);
        if (!axis || !swing) return desired;
        const auto carried = Multiply(*swing, previous);
        const auto correction = Multiply(desired, {-carried.x, -carried.y, -carried.z, carried.w});
        const auto roll = std::remainder(2 * std::atan2(Dot(*axis, {correction.x, correction.y, correction.z}), correction.w),
            2 * std::numbers::pi);
        const auto step = std::clamp(roll * follow, -maxStep, maxStep);
        return Multiply(AxisAngle(*axis, step), carried);
    }

    inline std::optional<LimbEdit> SolveLimb(const LimbPose& limb, Vec3 target, Vec3 pole,
                                          std::optional<Vec3> previousBend = {},
                                          std::optional<Quaternion> endRotation = {},
                                          std::optional<LimbRotation> turn = {}, bool followPole = true,
                                          const TorsoClearance* clearance = nullptr, const ArmDrag* armDrag = nullptr,
                                          std::optional<double> poleTravel = {})
    {
        const auto& [upper, lower, end] = limb.world;
        if (limb.middle == 0 || limb.localRotations.size() < 3 || limb.middle >= limb.localRotations.size() - 1)
            return std::nullopt;
        std::optional<Vec3> turnedSegment;
        if (turn) {
            if (turn->joint > 1) return std::nullopt;
            const auto& from = limb.world[turn->joint];
            const auto& to = limb.world[turn->joint + 1];
            const auto q = from.rotation;
            turnedSegment = Rotate(Multiply(turn->world, {-q.x, -q.y, -q.z, q.w}), to.position - from.position);
            if (!turnedSegment) return std::nullopt;
            pole = turn->joint == 0 ? upper.position + *turnedSegment : target - *turnedSegment;
        }
        bool bendLimited = false;
        std::optional<Vec3> preferredBend;
        // A direct arm bend deliberately steers the elbow around the planted wrist.
        // Still use the captured hinge below to orient both bones on the chosen plane.
        const bool directArmBend = followPole && endRotation && !turn && limb.hinge && !limb.hinge->leg;
        const auto dragAxis = armDrag ? Unit(target - upper.position) : std::nullopt;
        const auto carryBend = dragAxis ? CarryArmBend(*armDrag, *dragAxis, previousBend) : std::nullopt;
        if (directArmBend) {
            if (const auto axis = Unit(target - upper.position)) {
                const auto offset = pole - upper.position;
                const auto radial = offset - *axis * Dot(offset, *axis);
                const auto initial = lower.position - upper.position;
                const auto last = BendOnPlane(previousBend.value_or(initial), *axis);
                if (last) {
                    const auto reach = Length(initial) + Length(end.position - lower.position);
                    const auto radius = Length(initial - *axis * Dot(initial, *axis));
                    const auto lever = std::max(.05 * reach, radius);
                    auto bend = *last;
                    // A pointer near the shoulder-wrist line gives little information about swivel.
                    if (Length(radial) > lever * .12) {
                        const double swivel = std::atan2(Dot(Cross(*last, radial), *axis), Dot(*last, radial));
                        const double limit = poleTravel ? 1.5 * *poleTravel / lever : std::numbers::pi;
                        bend = *Rotate(AxisAngle(*axis, std::clamp(swivel, -limit, limit)), *last);
                    }
                    pole = upper.position + bend * reach;
                }
            }
        } else if (carryBend) {
            pole = upper.position + *carryBend *
                (Length(lower.position - upper.position) + Length(end.position - lower.position));
        } else if (limb.hinge) {
            const auto from = Unit(end.position - upper.position);
            const auto toward = Unit(target - upper.position);
            const auto normal = Rotate(upper.rotation, limb.hinge->localNormals[0]);
            if (from && toward && normal) {
                const auto preferred = Unit(Cross(*from, *normal));
                const auto swing = FromTo(*from, *toward);
                if (preferred && swing) {
                    auto bend = *Rotate(*swing, *preferred);
                    const auto neutralBend = bend;
                    // Near an about-face the shortest swing has no unique axis.
                    if (previousBend && Dot(*from, *toward) < -.98)
                        bend = Unit(*previousBend - *toward * Dot(*previousBend, *toward)).value_or(bend);
                    if (followPole || turn) {
                        const auto requestedPole = pole - upper.position;
                        if (const auto requested = Unit(requestedPole - *toward * Dot(requestedPole, *toward))) {
                            const auto alignment = Dot(bend, *requested);
                            bendLimited = alignment < limb.hinge->minBendAlignment;
                            // Keep the lateral component continuous when the mouse crosses behind the joint.
                            bend = *Unit(*requested + bend * (std::max(alignment, limb.hinge->minBendAlignment) - alignment));
                        }
                    }
                    if (turn && turn->maxBendStep > 0) {
                        // Near extension a small rotation can swing the end of
                        // the lower segment across the entire bend circle.
                        const auto last = previousBend ?
                            Unit(*previousBend - *toward * Dot(*previousBend, *toward)).value_or(neutralBend) : neutralBend;
                        const auto angle = std::atan2(Dot(Cross(last, bend), *toward), Dot(last, bend));
                        const auto step = std::clamp(angle, -turn->maxBendStep, turn->maxBendStep);
                        if (step != angle) bend = *Rotate(AxisAngle(*toward, step), last);
                    }
                    pole = upper.position + bend * (Length(lower.position - upper.position) + Length(end.position - lower.position));
                    preferredBend = neutralBend;
                }
            }
        }
        const TwoBoneInput input{upper.position, lower.position, end.position, target, pole, previousBend};
        auto result = SolveTwoBone(input);
        const auto parent = Unit(limb.parentRotation);
        if (!result || !parent) return std::nullopt;
        if (clearance) {
            const auto axis = Unit(result->tip - upper.position);
            const bool automaticBend = armDrag && axis;
            std::optional<Vec3> elbowGuide;
            const auto reach = Length(lower.position - upper.position) + Length(end.position - lower.position);
            if (automaticBend && Length(target - end.position) > reach * .02) {
                const auto up = Unit(clearance->chest - clearance->hips);
                const auto side = Dot(upper.position - clearance->chest, clearance->right) < 0 ? -1.0 : 1.0;
                if (up) {
                    const double raised = std::clamp(2 * Dot(target - upper.position, *up) / reach, 0., 1.);
                    elbowGuide = BendOnPlane(clearance->right * (side * std::lerp(.65, 1., raised)) +
                        (*up * -.85 + clearance->forward * .25) * (1 - raised), *axis);
                }
            }
            const auto scoreBend = [&](Vec3 joint, Vec3 bend) {
                const auto overlap = clearance->LimbOverlap(upper.position, joint, result->tip);
                return overlap + (elbowGuide ? .24 * (1 - Dot(*elbowGuide, bend)) : 0);
            };
            const auto initialScore = scoreBend(result->joint, result->bend);
            if (axis && initialScore > .002) {
                const auto along = upper.position + *axis * Dot(result->joint - upper.position, *axis);
                const auto radius = Length(result->joint - along);
                const auto maxTurn = automaticBend ? std::min(2.5 * armDrag->travel / reach, .8) :
                    poleTravel ? std::min(2.5 * *poleTravel / reach, .8) :
                    turn && turn->maxBendStep > 0 ? turn->maxBendStep : .80;
                const auto minAlignment = limb.hinge ? limb.hinge->minBendAlignment : -1.0;
                double best = initialScore;
                std::optional<Vec3> bestBend;
                for (const auto step : {-.25, .25, -.5, .5, -.75, .75, -1., 1.}) {
                    const auto angle = step * maxTurn;
                    const auto candidate = *Rotate(AxisAngle(*axis, angle), result->bend);
                    if (preferredBend && Dot(candidate, *preferredBend) < minAlignment - 1e-6) continue;
                    const auto joint = along + candidate * radius;
                    const auto score = scoreBend(joint, candidate) + .16 * angle * angle;
                    if (score >= best) continue;
                    best = score;
                    bestBend = candidate;
                }
                if (bestBend) {
                    auto steered = input;
                    steered.pole = upper.position + *bestBend * reach;
                    if (const auto edit = SolveTwoBone(steered)) {
                        result = edit;
                        bendLimited |= followPole;
                    }
                }
            }
        }
        const std::array segments{result->joint - upper.position, result->tip - result->joint};
        std::array swings{result->upperSwingWorld, result->lowerSwingWorld};
        if (limb.hinge) {
            if (const auto plane = Unit(Cross(result->bend, result->tip - upper.position))) {
                for (std::size_t i = 0; i < swings.size(); ++i)
                    swings[i] = FollowLimbPlane(swings[i], limb.world[i].rotation, limb.hinge->localNormals[i], segments[i], *plane);
            }
        }
        auto endpoint = endRotation.value_or(Multiply(swings[1], end.rotation));
        if (!armDrag && !endRotation && limb.hinge && !limb.hinge->leg) {
            // Moving the elbow around the bend plane can roll the forearm
            // sharply. Let the hand follow that roll only in part when dragging it.
            if (const auto axis = Unit(segments[1])) {
                const auto swing = result->lowerSwingWorld;
                const auto roll = Multiply(swings[1], {-swing.x, -swing.y, -swing.z, swing.w});
                endpoint = Multiply(AxisAngle(*axis, -.5 * RollSine(roll, *axis)), endpoint);
            }
        }
        std::array desired{Multiply(swings[0], upper.rotation), Multiply(swings[1], lower.rotation), endpoint};
        if (armDrag && !turn && !endRotation) {
            const auto& [lastUpper, lastLower, lastHand] = armDrag->previous;
            const auto lastLowerSegment = lastHand.position - lastLower.position;
            const double flexion = Length(Cross(*Unit(segments[0]), *Unit(segments[1])));
            if (flexion < .15) {
                // Near a straight or fully folded elbow its plane has little leverage.
                // Carry the prior roll through that singularity.
                const auto follow = flexion / .15;
                desired[0] = CarryArmRoll(lastUpper.rotation, lastLower.position - lastUpper.position,
                    segments[0], desired[0], std::numbers::pi, follow);
                desired[1] = CarryArmRoll(lastLower.rotation, lastLowerSegment, segments[1], desired[1], std::numbers::pi, follow);
            }
            const auto q = lower.rotation;
            const auto wristLocal = Multiply({-q.x, -q.y, -q.z, q.w}, end.rotation);
            const auto naturalHand = Multiply(desired[1], wristLocal);
            desired[2] = CarryArmRoll(lastHand.rotation, lastLowerSegment, segments[1], naturalHand, .30, .85);
        }
        if (turn) {
            // Preserve the requested roll, with a swing correction to keep both joints connected.
            const auto segment = segments[turn->joint];
            const auto align = FromTo(*turnedSegment, segment);
            if (!align) return std::nullopt;
            const auto rotated = Multiply(*align, turn->world);
            if (limb.hinge) {
                const auto axis = *Unit(segment);
                const auto natural = desired[turn->joint];
                const auto relative = Multiply(rotated, {-natural.x, -natural.y, -natural.z, natural.w});
                const auto roll = std::remainder(2 * std::atan2(Dot(axis, {relative.x, relative.y, relative.z}), relative.w),
                    2 * std::numbers::pi);
                const auto limitedRoll = std::clamp(roll, -limb.hinge->maxRoll, limb.hinge->maxRoll);
                bendLimited |= roll != limitedRoll;
                desired[turn->joint] = Multiply(AxisAngle(axis, limitedRoll), natural);
            } else desired[turn->joint] = rotated;
        }
        const std::array landmarks{std::size_t{0}, limb.middle, limb.localRotations.size() - 1};
        LimbEdit edit{{}, result->joint, result->tip, result->bend, desired[2], result->targetClamped || bendLimited, desired[1]};
        auto parentWorld = *parent;
        std::size_t landmark = 0;
        for (std::size_t i = 0; i < limb.localRotations.size(); ++i) {
            if (i == landmarks[landmark]) {
                const Quaternion inverse{-parentWorld.x, -parentWorld.y, -parentWorld.z, parentWorld.w};
                const auto local = Unit(Multiply(inverse, desired[landmark]));
                const auto world = Unit(desired[landmark]);
                if (!local || !world) return std::nullopt;
                edit.localRotations[landmark++] = *local;
                parentWorld = *world;
            } else {
                const auto world = Unit(Multiply(parentWorld, limb.localRotations[i]));
                if (!world) return std::nullopt;
                parentWorld = *world;
            }
        }
        return edit;
    }

    struct SupportedLimbEdit
    {
        LimbEdit limb;
        Quaternion support;
        bool adjusted{};
    };

    inline SupportedLimbEdit AssistLimbRoot(const LimbPose& baseline, LimbEdit primary, Transform support,
                                            Vec3 target, Vec3 pole, bool bend, std::optional<Vec3> previousBend,
                                            const TorsoClearance* clearance, const ArmDrag* armDrag = nullptr,
                                            std::optional<Quaternion> preferred = std::nullopt,
                                            std::optional<double> poleTravel = {})
    {
        SupportedLimbEdit result{primary, support.rotation};
        const auto endpointRotation = bend ? std::optional{baseline.world[2].rotation} : std::nullopt;
        const auto tipError = Length(primary.tip - target);
        const auto solve = [&](Quaternion rotation) {
            auto candidate = baseline;
            for (auto& joint : candidate.world) joint = *RotateAround(joint, support.position, rotation);
            candidate.parentRotation = Multiply(rotation, baseline.parentRotation);
            return SolveLimb(candidate, target, pole, previousBend, endpointRotation, {}, bend, clearance, armDrag, poleTravel);
        };
        const auto overlapAt = [&](Quaternion rotation, const LimbEdit& edit) {
            if (!clearance) return 0.0;
            const auto root = RotateAround(baseline.world[0], support.position, rotation);
            return clearance->LimbOverlap(root->position, edit.joint, edit.tip);
        };
        const auto overlap = overlapAt({}, primary);
        const bool needsClearance = overlap > .002 || tipError > .025;
        if (preferred) {
            // Keep a hand-placed shoulder unless yielding improves reach or clearance.
            if (!needsClearance) return result;
            const auto current = support.rotation;
            const auto relative = Multiply(*preferred, {-current.x, -current.y, -current.z, current.w});
            const double sign = relative.w < 0 ? -1 : 1;
            const Vec3 axis{sign * relative.x, sign * relative.y, sign * relative.z};
            const auto angle = 2 * std::atan2(Length(axis), std::abs(relative.w));
            const auto direction = Unit(axis).value_or(Vec3{1, 0, 0});
            for (const double portion : {1., .5, .25, .125, .0625}) {
                const auto rotation = AxisAngle(direction, angle * portion);
                const auto edit = solve(rotation);
                if (!edit) continue;
                const auto error = Length(edit->tip - target);
                const auto newOverlap = overlapAt(rotation, *edit);
                if (error > std::max(.025, tipError) || (bend && Length(edit->joint - primary.joint) > 1.0) ||
                    newOverlap > overlap + .002 || (error + .025 >= tipError && newOverlap + .002 >= overlap)) continue;
                return {*edit, Multiply(rotation, current), true};
            }
        }
        if (!clearance) return result;
        if (!armDrag && !needsClearance) return result;

        const auto up = Unit(clearance->chest - clearance->hips);
        if (!up) return result;
        const auto angle = std::clamp(.08 + .1 * std::sqrt(overlap) + .02 * tipError, .08, .21);
        std::array<Quaternion, 7> trials;
        std::size_t count = 0;
        Quaternion reachRotation{};
        if (armDrag) {
            const auto root = baseline.world[0].position;
            const auto clavicle = Unit(root - support.position);
            const auto from = Unit(baseline.world[2].position - root);
            const auto to = Unit(target - root);
            // Drive the shoulder end of the clavicle toward the reach. Rotation
            // around the clavicle's own shaft would leave the shoulder in place.
            if (const auto swing = clavicle && from && to ?
                    FromTo(*clavicle, *clavicle + (*to - *from) * .28) : std::nullopt) {
                const Vec3 turn{swing->x, swing->y, swing->z};
                if (const auto axis = Unit(turn)) {
                    const auto amount = std::min(.42, 2 * std::atan2(Length(turn), std::abs(swing->w)));
                    reachRotation = AxisAngle(*axis, amount);
                    trials[count++] = reachRotation;
                }
            }
        }
        if (needsClearance)
            for (const auto axis : {clearance->right, clearance->forward, *up})
                for (const auto turn : {-angle, angle}) trials[count++] = AxisAngle(axis, turn);
        const auto shoulderCost = [&](Quaternion rotation) {
            if (!armDrag) return .012;
            const auto relative = Multiply(rotation, {-reachRotation.x, -reachRotation.y, -reachRotation.z, reachRotation.w});
            return .45 * Length({relative.x, relative.y, relative.z});
        };
        double bestTip = tipError;
        double bestScore = overlap + (armDrag ? shoulderCost({}) : 0);
        for (const auto rotation : std::span(trials).first(count)) {
            const auto edit = solve(rotation);
            if (!edit) continue;
            const auto error = Length(edit->tip - target);
            const auto jointDrift = Length(edit->joint - primary.joint);
            if (error > std::max(.025, tipError) || error > bestTip + .025 || (bend && jointDrift > 1.0)) continue;
            const auto newOverlap = overlapAt(rotation, *edit);
            if (preferred && error + .025 >= tipError && newOverlap + .002 >= overlap) continue;
            const auto score = newOverlap + shoulderCost(rotation) + .009 * jointDrift;
            if (error + .025 >= bestTip && score >= bestScore) continue;
            bestTip = error;
            bestScore = score;
            result = {*edit, Multiply(rotation, support.rotation), true};
        }
        return result;
    }

    inline std::optional<LimbEdit> SolveAnchoredLimb(const LimbPose& limb, Transform anchor,
                                                     const TorsoClearance* clearance = nullptr)
    {
        auto edit = SolveLimb(limb, anchor.position, limb.world[1].position, {}, anchor.rotation, {}, false, clearance);
        if (edit && Length(edit->tip - anchor.position) <= 0.05) return edit;
        return std::nullopt;
    }

    inline std::optional<LimbEdit> SolveEndpointRotation(const LimbPose& limb, Quaternion current,
                                                        Quaternion requested, Quaternion currentLower, bool foot,
                                                        std::optional<Vec3> previousBend = {},
                                                        const TorsoClearance* clearance = nullptr)
    {
        const auto& lower = limb.world[1];
        const auto& end = limb.world[2];
        const auto goal = Unit(requested);
        const auto start = Unit(current);
        if (!goal || !start) return std::nullopt;

        auto change = Multiply(*goal, {-start->x, -start->y, -start->z, start->w});
        if (change.w < 0) change = {-change.x, -change.y, -change.z, -change.w};
        const Vec3 turn{change.x, change.y, change.z};
        const auto axis = Unit(turn);
        if (!axis) return std::nullopt;

        // Follow the current input increment so the limb keeps turning in the
        // same direction when the wrist or ankle passes a half-turn.
        const auto share = foot ? .55 : .8;
        const auto angle = 2 * std::atan2(Length(turn), change.w);
        // Sudden input can turn the endpoint freely without whipping the chain around it.
        auto followAngle = std::min(angle * share, .25);
        if (!foot) {
            // A small wrist turn can be handled by the wrist and twist bones alone.
            const auto base = lower.rotation;
            const auto local = Multiply({-base.x, -base.y, -base.z, base.w}, end.rotation);
            const auto neutral = Multiply(currentLower, local);
            const auto alignment = neutral.x * goal->x + neutral.y * goal->y + neutral.z * goal->z + neutral.w * goal->w;
            const auto wristAngle = 2 * std::acos(std::min(std::abs(alignment), 1.0));
            constexpr auto comfortableTurn = 25 * std::numbers::pi / 180;
            if (wristAngle <= comfortableTurn) return std::nullopt;
            followAngle = std::min(followAngle, wristAngle - comfortableTurn);
        }
        const auto follow = Multiply(AxisAngle(*axis, followAngle), currentLower);
        return SolveLimb(limb, end.position, lower.position, previousBend, *goal,
            LimbRotation{1, follow, 2 * followAngle}, true, clearance);
    }

}
