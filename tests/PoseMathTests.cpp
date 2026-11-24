#include "../src/core/FingerPose.h"
#include "../src/core/LimbPose.h"
#include "../src/core/EndpointModel.h"
#include "../src/core/Breathing.h"
#include "../src/core/GrabButton.h"
#include "../src/core/PoseSerialization.h"
#include "../src/core/PoseHistory.h"
#include "../src/core/DragView.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <sstream>
#include <vector>

using namespace GModPose;

namespace
{
    int assertions = 0;
    void Check(bool condition, const char* name)
    {
        ++assertions;
        if (!condition) { std::cerr << "FAIL: " << name << '\n'; std::exit(1); }
    }
    bool Near(double a, double b, double tolerance = 1e-7) { return std::abs(a - b) <= tolerance; }
    bool Near(Vec3 a, Vec3 b, double tolerance = 1e-7) { return Length(a - b) <= tolerance; }
    TwoBoneInput Arm()
    {
        return {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {1, 1, 0}, {0, 1, 1}, std::nullopt};
    }

    void RotationAndSpace()
    {
        const auto turn = FromTo({1, 0, 0}, {0, 1, 0});
        Check(turn && Near(*Rotate(*turn, {2, 0, 0}), {0, 2, 0}), "quarter rotation");
        // Near-opposite vectors still carry a small steering angle.
        const auto opposite = FromTo({1, 0, 0}, {-1, 0, 0});
        Check(opposite && Near(*Rotate(*opposite, {3, 0, 0}), {-3, 0, 0}), "antiparallel rotation");
        const auto nearlyOpposite = FromTo({1, 0, 0}, {-1, 1e-6, 0});
        Check(nearlyOpposite && Near(*Rotate(*nearlyOpposite, {1, 0, 0}), *Unit(Vec3{-1, 1e-6, 0}), 1e-10),
            "near-antiparallel rotation retains small direction changes");
        Check(!FromTo({}, {1, 0, 0}), "zero segment rejects rotation");
        // Round-trip positions and orientations through a rotated parent.
        const Transform parent{{10, 20, 30}, *turn};
        const Transform local{{3, 0, 0}, *opposite};
        const auto world = Compose(parent, local);
        Check(world && Near(world->position, {10, 23, 30}), "parent space translation");
        const auto recovered = RelativeTo(parent, *world);
        Check(recovered && Near(recovered->position, local.position), "relative position roundtrip");
        Check(Near(*Rotate(recovered->rotation, {1, 2, 3}), *Rotate(local.rotation, {1, 2, 3})),
            "relative rotation roundtrip");
        const auto twice = Multiply(*turn, *turn);
        Check(Near(*Rotate(twice, {1, 0, 0}), {-1, 0, 0}), "quaternion composition order");
        Check(!Rotate({0, 0, 0, 0}, {1, 0, 0}), "invalid quaternion rejected");
    }

    void InverseKinematics()
    {
        auto arm = Arm();
        arm.target = {1.2, .4, .3};
        const auto solved = SolveTwoBone(arm);
        Check(solved && !solved->targetClamped && Near(solved->tip, arm.target), "reachable target reached");
        Check(Near(Length(solved->joint - arm.root), 1), "upper length preserved");
        Check(Near(Length(solved->tip - solved->joint), 1), "lower length preserved");
        Check(Near(*Rotate(solved->upperSwingWorld, arm.joint - arm.root), solved->joint - arm.root), "upper swing matches geometry");
        Check(Near(*Rotate(solved->lowerSwingWorld, arm.tip - arm.joint), solved->tip - solved->joint), "lower swing matches geometry");
        // Exercise both ends of the reach interval, including a fully folded chain.
        arm.target = {100, 0, 0};
        auto result = SolveTwoBone(arm);
        Check(result && result->targetClamped && Near(result->tip, {2, 0, 0}), "far target clamps without stretching");
        arm.target = arm.root;
        result = SolveTwoBone(arm);
        Check(result && result->targetClamped && Near(Length(result->tip - result->joint), 1), "equal-length folded singularity");
        arm.joint = {2, 0, 0}; arm.tip = {2, 1, 0}; arm.target = {.1, 0, 0};
        result = SolveTwoBone(arm);
        Check(result && Near(Length(result->tip - arm.root), 1) && Near(Length(result->tip - result->joint), 1), "unequal-length minimum reach");
        // A collinear pole leaves the bend ambiguous. Reuse the last bend when available.
        arm = Arm(); arm.target = {1, 0, 0}; arm.pole = {3, 0, 0}; arm.previousBend = Vec3{0, 0, 1};
        result = SolveTwoBone(arm);
        Check(result && result->joint.z > 0 && Near(result->joint.y, 0), "collinear pole uses prior bend");
        arm.previousBend.reset();
        result = SolveTwoBone(arm);
        Check(result && IsFinite(result->joint), "collinear input has deterministic fallback");
        arm.joint = arm.root;
        Check(!SolveTwoBone(arm), "zero-length chain rejected");

        // Carry the previous bend around a full orbit to catch sudden elbow flips.
        arm = Arm(); arm.pole = {0, 0, 3};
        Vec3 previousJoint{};
        bool first = true;
        for (int i = 0; i <= 720; ++i) {
            const double angle = i * std::numbers::pi / 360;
            arm.target = {1.3 * std::cos(angle), 1.3 * std::sin(angle), .15};
            result = SolveTwoBone(arm);
            Check(result.has_value(), "continuous target solution exists");
            Check(Near(Length(result->joint), 1) && Near(Length(result->tip - result->joint), 1), "sweep bone lengths preserved");
            if (!first) Check(Length(result->joint - previousJoint) < .03, "elbow continuity across full orbit");
            previousJoint = result->joint; arm.previousBend = result->bend; first = false;
        }
    }

    void NaturalLimbBends()
    {
        LimbPose leg{{Transform{{0, 0, 0}, {}}, Transform{{0, .6, -.8}, {}}, Transform{{0, 0, -1.6}, {}}},
            {}, {{}, {}, {}}, 1};
        leg.hinge = CaptureLimbHinge(leg.world, {0, 1, 0}, true);
        const auto foot = leg.world[2];
        // A knee can steer sideways around a planted foot while retaining its forward hinge.
        const auto reversed = SolveLimb(leg, foot.position, {0, -8, -.8}, {}, foot.rotation);
        Check(reversed && reversed->limited && reversed->joint.y > .5 && Near(reversed->tip, foot.position),
            "dragging a knee behind its joint keeps the natural bend and plants the foot");
        const auto lateral = SolveLimb(leg, foot.position, {3, 0, -.8}, {}, foot.rotation);
        Check(lateral && lateral->limited && lateral->joint.x > .2 && lateral->joint.y > .1 &&
            Near(lateral->tip, foot.position), "the bend limit allows controlled lateral adjustment");
        const auto plane = *Unit(Cross(lateral->bend, lateral->tip - leg.world[0].position));
        const auto upper = lateral->localRotations[0];
        const auto lower = Multiply(upper, lateral->localRotations[1]);
        Check(Near(*Rotate(upper, leg.hinge->localNormals[0]), plane) &&
            Near(*Rotate(lower, leg.hinge->localNormals[1]), plane), "both limb rotations align with the solved bend plane");

        // Shin twist saturates at the roll limit without shifting either joint.
        const auto shin = *Unit(foot.position - leg.world[1].position);
        const auto twist = SolveLimb(leg, foot.position, leg.world[1].position, {}, foot.rotation,
            LimbRotation{1, AxisAngle(shin, std::numbers::pi * .6)});
        Check(twist && twist->limited && Near(twist->tip, foot.position) && Near(twist->joint, leg.world[1].position),
            "excessive shin twist saturates without moving either joint");
        const auto twistedLower = Multiply(twist->localRotations[0], twist->localRotations[1]);
        const auto expected = AxisAngle(shin, leg.hinge->maxRoll);
        Check(Near(*Rotate(twistedLower, leg.hinge->localNormals[1]), *Rotate(expected, leg.hinge->localNormals[1])),
            "rotation remains available within the hinge's roll limit");

        LimbPose arm{{Transform{{0, 0, 0}, {}}, Transform{{1, 0, 0}, {}}, Transform{{2, 0, 0}, {}}},
            {}, {{}, {}, {}}, 1};
        arm.hinge = CaptureLimbHinge(arm.world, {0, -1, 0}, false);
        const auto bent = SolveLimb(arm, {1.5, 0, 0}, {0, 1, 0}, {}, {}, {}, false);
        Check(bent && bent->joint.y < -.5 && Near(bent->tip, {1.5, 0, 0}),
            "a fully extended arm bends according to its reference direction when dragging the wrist");
    }

    void PlantedWristBend()
    {
        for (const double side : {-1., 1.}) {
            const Vec3 shoulder{side * 16, 0, 30}, wrist{side * 8, -5, 46};
            const auto axis = *Unit(wrist - shoulder);
            const auto outward = *BendOnPlane({side, 0, 0}, axis);
            const auto center = (shoulder + wrist) * .5;
            const auto elbow = center + outward * 8;
            const auto handRotation = AxisAngle(axis, .4);
            LimbPose arm{{Transform{shoulder, {}}, Transform{elbow, {}}, Transform{wrist, handRotation}},
                {}, {{}, {}, handRotation}, 1};
            arm.hinge = CaptureLimbHinge(arm.world, outward, false);
            Check(arm.hinge.has_value(), "a raised arm has a captured hinge");
            // A planted hand keeps its world orientation while the elbow circles the wrist axis.
            std::optional<Vec3> previous;
            for (int step = 0; step <= 18; ++step) {
                const auto swivel = AxisAngle(axis, side * step * std::numbers::pi / 18);
                const auto target = center + *Rotate(swivel, outward) * 8;
                const auto edit = SolveLimb(arm, wrist, target, previous, handRotation);
                Check(edit && !edit->limited && Near(edit->joint, target) && Near(edit->tip, wrist),
                    "direct arm bend can steer through a half-turn with the wrist fixed");
                const auto upper = edit->localRotations[0];
                const auto lower = Multiply(upper, edit->localRotations[1]);
                const auto hand = Multiply(lower, edit->localRotations[2]);
                Check(Near(*Rotate(upper, elbow - shoulder), target - shoulder) &&
                    Near(*Rotate(lower, wrist - elbow), wrist - target) &&
                    Near(*AngularCorrection(hand, handRotation, 1, 10), {}),
                    "arm reconstruction maintains bone lengths and wrist orientation through the swivel");
                previous = edit->bend;
            }
            // Cross the shoulder-wrist axis inside the pole dead zone and then leave it.
            auto last = outward;
            for (const double distance : {.01, -.01, -.96}) {
                const auto edit = SolveLimb(arm, wrist, center + outward * distance, last, handRotation,
                    {}, true, nullptr, nullptr, .02);
                Check(edit && Near(edit->bend, last) && Near(edit->tip, wrist),
                    "crossing near the shoulder-wrist line preserves an authored elbow swivel");
                last = edit->bend;
            }
            const auto outside = SolveLimb(arm, wrist, center - outward * 2, last, handRotation,
                {}, true, nullptr, nullptr, 1.04);
            Check(outside && Dot(outside->bend, last) > .97, "leaving the pole dead zone turns the elbow continuously");

            const auto segment = *Unit(elbow - shoulder);
            // A fresh elbow grip should start with the roll already authored on the arm.
            const auto authored = AxisAngle(segment, .92);
            arm.world[0].rotation = authored;
            arm.hinge = HingeAtGrip(arm.world, arm.hinge);
            const auto grab = SolveLimb(arm, wrist, elbow, {}, handRotation);
            Check(grab && Near(*AngularCorrection(authored, grab->localRotations[0], 1, 10), {}),
                "a new elbow grip preserves axial roll already authored on a bent arm");
        }
    }

    void LimbCollisionClearance()
    {
        const auto body = MakeTorsoClearance({0, 0, 0}, {0, 0, 30}, {-16, 0, 30}, {16, 0, 30});
        Check(body && body->Penetration({0, 0, 18}) > .99 && body->Penetration({18, 0, 18}) == 0,
            "the torso proxy distinguishes points inside and outside the body");
        const LimbPose arm{{Transform{{16, 0, 30}, {}}, Transform{{10, 0, 12}, {}}, Transform{{23, 10, 22}, {}}},
            {}, {{}, {}, {}}, 1};
        const auto& [shoulder, elbow, hand] = arm.world;
        const auto ordinary = SolveLimb(arm, hand.position, elbow.position);
        const auto protectedArm = SolveLimb(arm, hand.position, elbow.position, {}, hand.rotation, {}, true, &*body);
        Check(ordinary && protectedArm && Near(protectedArm->tip, hand.position) &&
            Length(protectedArm->joint - elbow.position) > .1 &&
            body->LimbOverlap(shoulder.position, protectedArm->joint, protectedArm->tip) <
                body->LimbOverlap(shoulder.position, ordinary->joint, ordinary->tip),
            "steering an elbow clear of the torso keeps the hand planted");
        Check(Near(Length(protectedArm->joint - shoulder.position), Length(elbow.position - shoulder.position)) &&
            Near(Length(protectedArm->tip - protectedArm->joint), Length(hand.position - elbow.position)),
            "collision steering preserves both bone lengths");
        const Transform clavicle{{10, 0, 33}, {}};
        const Vec3 far{70, 0, 30};
        const auto before = SolveLimb(arm, far, elbow.position, {}, {}, {}, false, &*body);
        Check(before.has_value(), "the unassisted arm produces a reachable position");
        const auto supported = AssistLimbRoot(arm, *before, clavicle, far, elbow.position, false, {}, &*body);
        Check(supported.adjusted && Length(supported.limb.tip - far) < Length(before->tip - far),
            "a bounded clavicle turn can improve reach without overwriting the endpoint target");
        const auto fixed = AssistLimbRoot(arm, *protectedArm, clavicle, hand.position, elbow.position, true, {}, &*body);
        Check(Near(fixed.limb.tip, hand.position) && Length(fixed.limb.joint - protectedArm->joint) <= 1,
            "the clavicle stays subordinate to a planted hand and explicit elbow target");

        // An elbow inside the head is a collision. A hand reaching behind it can be valid.
        auto withHead = *body;
        withHead.head = TorsoClearance::Head{{0, 0, 50}, 6};
        Check(withHead.LimbOverlap({16, 0, 30}, {0, 0, 50}, {-8, -6, 50}) > .9 &&
            Near(withHead.LimbOverlap({16, 0, 30}, {20, -6, 50}, {8, -6, 50}), 0),
            "head clearance distinguishes an elbow inside the head from a hand resting behind it");
    }

    void AbsoluteWristSolve()
    {
        const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() /
            "assets/SKSE/Plugins/GModPose/ArmPrior.bin";
        EndpointReference rig{};
        rig.context[0] = {0, 0, -22};
        rig.context[1] = {0, 0, 29};
        rig.clavicle = Transform{{3, 0, 12}, {}};
        rig.shoulderParent = *rig.clavicle;
        rig.joints = {Transform{{14, 0, 12}, {}}, Transform{{36, 0, 12}, {}}, Transform{{52, 0, 12}, {}}};
        rig.clearance = MakeTorsoClearance(rig.context[0], {0, 0, 12}, {-14, 0, 12}, {14, 0, 12});

        EndpointModel model;
        const Vec3 target{7, -11, -17};
        Check(model.Predict(rig, target).View().empty(), "unloaded wrist model gives no candidates");
        Check(model.Load(path), "the shipped GPU trainer format loads");
        const auto predict = [&](Vec3 goal) {
            const auto candidates = model.Predict(rig, goal);
            Check(candidates.View().size() == EndpointModel::modes, "the wrist model outputs three full arm poses");
            return SolveEndpoint(rig, goal, candidates.View());
        };
        const auto behind = predict(target);
        Check(behind && Near(behind->joints[2].position, target) && !behind->limited,
            "absolute wrist solve reaches a target behind the torso");
        // Rebuild each attachment from its bind offset to check exact bone lengths.
        const auto checkLinks = [&](const EndpointSolution& solved) {
            for (std::size_t i = 0; i < 2; ++i) {
                const auto bind = RelativeTo(rig.joints[i], rig.joints[i + 1]);
                const auto end = Compose(solved.joints[i], *bind);
                Check(end && Near(end->position, solved.joints[i + 1].position),
                    "absolute rotations preserve exact attachment and segment length");
            }
        };
        checkLinks(*behind);
        const auto overhead = predict({17, 4, 44});
        Check(overhead && Near(overhead->joints[2].position, {17, 4, 44}), "absolute wrist solve reaches overhead");
        // An authored wrist flex follows the solved forearm after a positional edit.
        const auto wristFlex = AxisAngle({0, 1, 0}, .65);
        const auto wristPoses = model.Predict(rig, target);
        const auto bent = SolveEndpoint(rig, target, wristPoses.View(), {}, wristFlex);
        Check(bent && Near(bent->joints[2].position, target) &&
            Near(*AngularCorrection(wristFlex, RelativeTo(bent->joints[1], bent->joints[2])->rotation, 1, 10), {}),
            "positioning a wrist preserves its bend relative to the forearm");
        // Revisit the behind-body target after an overhead solve and a flexed wrist solve.
        const auto again = predict(target);
        Check(again && Near(again->joints[1].position, behind->joints[1].position), "wrist solve ignores previous bend");
        for (std::size_t i = 0; i < 3; ++i)
            Check(Near(*AngularCorrection(again->joints[i].rotation, behind->joints[i].rotation, 1, 10), {}),
                "wrist solve ignores previous joint rotations");
        const auto firstTwist = EndpointTwistFrame(rig, *behind, 1);
        const auto repeatedTwist = EndpointTwistFrame(rig, *again, 1);
        Check(firstTwist && repeatedTwist && Near(firstTwist->proximal.angle, repeatedTwist->proximal.angle) &&
            Near(firstTwist->distal.angle, repeatedTwist->distal.angle),
            "twist helpers are reconstructed from the reference rather than previous roll");

        // Without weights, the reference-only solve still reaches and can enlist the clavicle.
        const auto far = SolveEndpoint(rig, {10000, 0, 0});
        Check(far && far->limited, "absolute fallback marks an unreachable wrist");
        checkLinks(*far);
        const Vec3 farUpper{41, 0, 41};
        const auto reaching = SolveEndpoint(rig, farUpper);
        Check(reaching && Near(reaching->joints[2].position, farUpper) &&
            !Near(*AngularCorrection(rig.clavicle->rotation, reaching->clavicle, 1, 10), {}),
            "a fresh hand solve moves the clavicle to reach beyond a fixed shoulder");
        Check(!model.Load(path.parent_path() / "missing-arm-prior.bin") && model.Predict(rig, target).View().empty(),
            "a missing model falls back to a solve from fixed references");
        const auto procedural = SolveEndpoint(rig, target);
        Check(procedural && Near(procedural->joints[2].position, target), "reference-only fallback reaches the wrist target");

        // Two nearly tied elbow proposals should blend without jumping between poses.
        rig.clavicle.reset();
        rig.clearance.reset();
        const auto base = SolveEndpoint(rig, target);
        const auto root = base->joints[0].position;
        const auto axis = *Unit(target - root);
        const auto proposal = [&](double angle) {
            const auto turn = AxisAngle(axis, angle);
            const auto joint = *RotateAround(base->joints[1], root, turn);
            return EndpointCandidate{root, joint.position,
                {Multiply(turn, base->joints[0].rotation), joint.rotation, Multiply(turn, base->joints[2].rotation)}, 0};
        };
        std::array alternatives{proposal(.45), proposal(-.45)};
        alternatives[0].logit = .02;
        const auto preferFirst = SolveEndpoint(rig, target, alternatives);
        alternatives[0].logit = -.02;
        const auto preferSecond = SolveEndpoint(rig, target, alternatives);
        Check(preferFirst && preferSecond && Near(preferSecond->joints[2].position, target) &&
            Length(preferSecond->joints[1].position - preferFirst->joints[1].position) < .2,
            "nearly tied bend proposals transition smoothly with the same wrist target");
    }

    void LearnedAnkleAndChestStance()
    {
        const auto directory = std::filesystem::path(__FILE__).parent_path().parent_path() / "assets/SKSE/Plugins/GModPose";
        EndpointModel model;
        Check(!model.Load(directory / "ArmPrior.bin", true), "ankle solver rejects wrist weights");
        Check(model.Load(directory / "LegPrior.bin", true), "shipped ankle weights load");

        // Exercise mirrored raised-foot and planted-foot cases using the shipped weights.
        const std::array<Transform, 2> pins{Transform{{-13, 1, 7}, AxisAngle({0, 0, 1}, .3)},
            Transform{{16, 6, 7}, AxisAngle({0, 0, 1}, -.4)}};
        for (std::size_t side = 0; side < pins.size(); ++side) {
            const double sign = side == 0 ? -1 : 1;
            EndpointReference rig{};
            rig.frame.position = {0, 0, 69};
            rig.context = {Vec3{0, -6, 91}, pins[1 - side].position};
            rig.shoulderParent = rig.frame;
            rig.joints = {Transform{{sign * 7, 0, 69}, {}}, Transform{{sign * 10, 4, 38}, {}},
                Transform{{sign * 12, 0, 7}, {}}};
            rig.left = side == 0;
            rig.leg = true;
            const Vec3 lift{-sign * 12, 18, 34};
            const auto candidates = model.Predict(rig, lift);
            Check(candidates.count == EndpointModel::modes, "model proposes full poses for a raised crossed ankle");
            const auto lifted = SolveEndpoint(rig, lift, candidates.View());
            Check(lifted && Near(lifted->joints[2].position, lift), "learned ankle steering keeps a lifted foot on target");
            Check(lifted->joints[1].position.y > 0, "a lifted foot bends the knee forward");
            Check(EndpointTwistFrame(rig, *lifted, 1).has_value(), "lifted ankle redistributes calf twist from the rig");

            // Keep flex in calf space when a raised ankle is repositioned.
            const auto flex = AxisAngle({1, 0, 0}, .75);
            const auto flexed = SolveEndpoint(rig, lift, candidates.View(), {}, flex);
            Check(flexed && Near(flexed->joints[2].position, lift) &&
                Near(*AngularCorrection(flex, RelativeTo(flexed->joints[1], flexed->joints[2])->rotation, 1, 10), {}),
                "moving a flexed ankle preserves its bend relative to the calf");

            // Lowered chest moves the pelvis. The world-space ankle pins stay fixed.
            const Vec3 down{0, 0, -10};
            rig.frame.position = rig.frame.position + down;
            rig.shoulderParent.position = rig.shoulderParent.position + down;
            rig.context[0] = rig.context[0] + down;
            for (auto& joint : rig.joints) joint.position = joint.position + down;
            const auto pin = pins[side];
            const auto stance = model.Predict(rig, pin.position);
            const auto planted = SolveEndpoint(rig, pin.position, stance.View(), pin.rotation);
            Check(planted && Near(planted->joints[2].position, pin.position) &&
                Near(*AngularCorrection(pin.rotation, planted->joints[2].rotation, 1, 10), {}),
                "learned chest stance preserves the position and orientation of a planted foot");
            Check(Near(Length(planted->joints[1].position - planted->joints[0].position),
                       Length(rig.joints[1].position - rig.joints[0].position)) &&
                Near(Length(planted->joints[2].position - planted->joints[1].position),
                     Length(rig.joints[2].position - rig.joints[1].position)),
                "chest stance preserves both leg segment lengths");
        }
        // Small head turns recruit the neck. Large turns saturate the neck contribution.
        const auto turn = AxisAngle({0, 0, 1}, .8);
        Check(Near(*AngularCorrection({}, NeckFollow({}, {}, turn), 1, 10), {0, 0, .28}),
            "a head turn shares part of its angle with the neck");
        const auto full = AxisAngle({1, 0, 0}, 2.0);
        Check(Near(*AngularCorrection({}, NeckFollow({}, {}, full), 1, 10), {.40, 0, 0}),
            "neck assistance is bounded for a large head turn");
    }

    void CoordinatedArmDrag()
    {
        auto body = MakeTorsoClearance({0, 0, 0}, {0, 0, 30}, {-16, 0, 30}, {16, 0, 30});
        Check(body.has_value(), "the arm drag has a torso frame");
        const Vec3 shoulder{16, 0, 30}, elbow{18, 2, 11}, hand{16, 6, -5};
        const auto initialUpper = *FromTo({0, 0, -1}, elbow - shoulder);
        const auto initialLower = *FromTo({0, 0, -1}, hand - elbow);
        const LimbPose arm{{Transform{shoulder, initialUpper}, Transform{elbow, initialLower},
            Transform{hand, Multiply(AxisAngle(*Unit(hand - elbow), .30), initialLower)}},
            {}, {{}, {}, {}}, 1, CaptureLimbHinge({Transform{shoulder, initialUpper}, Transform{elbow, initialLower},
                Transform{hand, initialLower}}, {0, 1, 0}, false)};
        Check(arm.hinge.has_value(), "the dragged arm has a captured hinge");
        auto previous = arm.world;
        auto oldTarget = hand;
        std::optional<Vec3> bend;
        double assistedOverlap = 0, oldOverlap = 0;
        // Drag through several reach directions and return to the initial wrist position.
        for (const Vec3 destination : {Vec3{12, 17, 15}, Vec3{-9, 15, 26}, Vec3{8, 18, 35}, hand}) {
            const auto start = oldTarget;
            for (int step = 1; step <= 36; ++step) {
                const auto target = start + (destination - start) * (step / 36.0);
                const ArmDrag drag{previous, Length(target - oldTarget)};
                const auto edit = SolveLimb(arm, target, elbow, bend, {}, {}, false, &*body, &drag);
                const auto oldSolve = SolveLimb(arm, target, elbow, {}, {}, {}, false, &*body);
                Check(edit && oldSolve && Near(edit->tip, target), "the hand reaches each across-body drag target");
                const auto newLower = edit->tip - edit->joint;
                const auto swing = *FromTo(previous[2].position - previous[1].position, newLower);
                const auto transported = Multiply(swing, previous[2].rotation);
                Check(Length(*AngularCorrection(transported, edit->endRotation, 1, 10)) <= .300001,
                    "a hand drag bounds axial wrist rotation at each step");
                const auto upper = edit->localRotations[0];
                const auto lower = Multiply(upper, edit->localRotations[1]);
                const auto& [initialRoot, initialElbow, initialHand] = arm.world;
                const auto upperReference = Fingers::Conjugate(initialRoot.rotation);
                const auto lowerReference = Fingers::Conjugate(initialElbow.rotation);
                Check(Near(*Rotate(Multiply(upper, upperReference), initialElbow.position - shoulder), edit->joint - shoulder) &&
                    Near(*Rotate(Multiply(lower, lowerReference), initialHand.position - initialElbow.position), newLower),
                    "distributed roll preserves both segment positions");
                assistedOverlap += body->LimbOverlap(shoulder, edit->joint, edit->tip);
                oldOverlap += body->LimbOverlap(shoulder, oldSolve->joint, oldSolve->tip);
                previous = {Transform{shoulder, upper}, Transform{edit->joint, lower}, Transform{edit->tip, edit->endRotation}};
                bend = edit->bend;
                oldTarget = target;
            }
        }
        Check(assistedOverlap < oldOverlap, "continuous elbow steering reduces overlap across the chest");
        const Vec3 sideReach{28, 12, 17};
        const ArmDrag motion{arm.world, Length(sideReach - hand)};
        const auto edit = SolveLimb(arm, sideReach, elbow, {}, {}, {}, false, &*body, &motion);
        Check(edit.has_value(), "the primary arm solve can reach out to the side");
        const auto supported = AssistLimbRoot(arm, *edit, {{9, 0, 30}, {}}, sideReach, elbow,
            false, {}, &*body, &motion);
        Check(supported.adjusted && Near(supported.limb.tip, sideReach),
            "a natural shoulder turn supports a reachable hand position");
        const std::array straightAhead{Transform{{16, 0, 30}, {}}, Transform{{19, -3, 11}, {}}, Transform{{16, 0, -4}, {}}};
        const LimbPose sagittal{straightAhead, {}, {{}, {}, {}}, 1, CaptureLimbHinge(straightAhead, {1, -1, 0}, false)};
        const Transform pivot{{4, 0, 30}, {}};
        const Vec3 high{16, 12, 46};
        const ArmDrag lift{straightAhead, Length(high - straightAhead[2].position)};
        const auto initial = SolveLimb(sagittal, high, straightAhead[1].position, {}, {}, {}, false, &*body, &lift).value();
        const auto elevated = AssistLimbRoot(sagittal, initial, pivot, high, straightAhead[1].position, false, {}, &*body, &lift);
        const auto raisedRoot = RotateAround(straightAhead[0], pivot.position, elevated.support).value();
        Check(raisedRoot.position.z > straightAhead[0].position.z + 1 && Near(elevated.limb.tip, high),
            "raising a hand in front lifts the shoulder end of a lateral clavicle");

        const std::array posed{Transform{{14.8, 0, 31.5}, {}}, Transform{{22.5, -3.1, 52.8}, {}},
            Transform{{6.9, -6.0, 50.1}, {}}};
        const LimbPose overhead{posed, {}, {{}, {}, {}}, 1, CaptureLimbHinge(posed, {1, -1, 0}, false)};
        auto held = posed;
        std::optional<Vec3> swivel;
        for (int i = 0; i < 60; ++i) {
            const auto target = posed[2].position + Vec3{0, i % 2 == 0 ? .005 : 0, 0};
            const ArmDrag nudge{held, .005};
            const auto e = SolveLimb(overhead, target, posed[1].position, swivel, {}, {}, false, &*body, &nudge).value();
            held = {Transform{posed[0].position, e.localRotations[0]}, Transform{e.joint, e.lowerRotation},
                Transform{e.tip, e.endRotation}};
            swivel = e.bend;
        }
        Check(Near(held[1].position, posed[1].position) && Near(held[2].position, posed[2].position),
            "small wrist motions that return to a raised pose do not walk the elbow around the shoulder");
    }

    void ClavicleRecovery()
    {
        const Vec3 pivot{10, 0, 28};
        const Quaternion captured{};
        // Start from an authored clavicle lift. It should yield only when reach demands it.
        const auto raised = AxisAngle({0, 1, 0}, -1.25);
        const std::array neutral{Transform{{16, 0, 28}, {}}, Transform{{24, 4, 17}, {}}, Transform{{20, 12, 8}, {}}};
        LimbPose edited{neutral, raised, {{}, {}, {}}, 1};
        edited.hinge = CaptureLimbHinge(neutral, {0, 1, 0}, false);
        for (auto& joint : edited.world) joint = *RotateAround(joint, pivot, raised);
        const auto& [shoulder, elbow, wrist] = edited.world;

        const Vec3 target{19, 10, 11};
        const auto primary = SolveLimb(edited, target, elbow.position);
        Check(primary && Near(primary->tip, target), "the edited shoulder can reach the hand drag target");
        const Transform clavicle{pivot, raised};
        const ArmDrag drag{edited.world, Length(target - wrist.position)};
        const auto restored = AssistLimbRoot(edited, *primary, clavicle, target, elbow.position,
            false, {}, nullptr, &drag, captured);
        const auto sameRotation = [](Quaternion left, Quaternion right) {
            return Near(Length(*AngularCorrection(left, right, 1, 10)), 0);
        };
        Check(!restored.adjusted && sameRotation(restored.support, raised) &&
            Near(restored.limb.tip, target), "a reachable wrist keeps the manually placed clavicle");
        const auto body = MakeTorsoClearance({0, 0, 0}, {0, 0, 30}, {-16, 0, 30}, {16, 0, 30});
        Check(body.has_value(), "clavicle recovery has a torso clearance frame");
        const auto clear = AssistLimbRoot(edited, *primary, clavicle, target, elbow.position,
            false, {}, &*body, &drag, captured);
        Check(!clear.adjusted && sameRotation(clear.support, raised) && Near(clear.limb.tip, target),
            "a clear arm keeps the authored shoulder during a hand edit");

        const auto length = Length(elbow.position - shoulder.position) + Length(wrist.position - elbow.position);
        const auto outward = *Unit(shoulder.position - neutral[0].position);
        const auto far = shoulder.position + outward * (length - 1.2);
        Check(Length(far - neutral[0].position) > length, "the far target rules out a full clavicle reset");
        const auto extended = SolveLimb(edited, far, elbow.position);
        Check(extended && Near(extended->tip, far), "the elevated shoulder can still reach the far target");
        const auto partial = AssistLimbRoot(edited, *extended, clavicle, far, elbow.position,
            false, {}, nullptr, nullptr, captured);
        Check(!partial.adjusted && sameRotation(partial.support, raised) && Near(partial.limb.tip, far),
            "the clavicle holds when the hand needs its extra reach");
        const auto towardNeutral = neutral[0].position - outward * (length - .5);
        const auto tooFar = SolveLimb(edited, towardNeutral, elbow.position).value();
        const auto yielded = AssistLimbRoot(edited, tooFar, clavicle, towardNeutral, elbow.position,
            false, {}, nullptr, nullptr, captured);
        Check(yielded.adjusted && Near(yielded.limb.tip, towardNeutral), "a manual clavicle can yield to reach the hand");
    }

    void EndpointRotationIK()
    {
        // Both wrist and ankle turns keep the selected pivot fixed as the limb follows.
        for (const bool foot : {false, true}) {
            LimbPose limb{{Transform{{0, 0, 0}, {}}, Transform{{0, .6, -.8}, {}}, Transform{{0, 0, -1.6}, {}}},
                {}, {{}, {}, {}}, 1};
            const auto& [root, joint, end] = limb.world;
            limb.hinge = CaptureLimbHinge(limb.world, {0, 1, 0}, foot);
            Check(limb.hinge.has_value(), "rotation constraints capture from the limb pose");

            const auto idle = SolveEndpointRotation(limb, end.rotation, {0, 0, 0, -1}, joint.rotation, foot);
            Check(!idle, "unchanged rotation skips an IK edit");
            const auto spanAxis = *Unit(end.position - root.position);
            if (!foot) {
                const auto small = AxisAngle(spanAxis, .3);
                Check(!SolveEndpointRotation(limb, end.rotation, small, joint.rotation, false),
                    "a small wrist turn leaves the forearm and elbow in place");
            }
            const auto turn = AxisAngle(spanAxis, .65);
            const auto turned = SolveEndpointRotation(limb, end.rotation, turn, joint.rotation, foot);
            Check(turned && Near(turned->tip, end.position) && Length(turned->joint - joint.position) > .06,
                "rotating a planted wrist or foot steers the elbow or knee");
            Check(Near(*AngularCorrection(turned->endRotation, turn, 1, 10), {}),
                "an assisted endpoint turn reaches the requested world orientation");
            const auto lowerWorld = Multiply(turned->localRotations[0], turned->localRotations[1]);
            Check(Near(*Rotate(turned->localRotations[0], joint.position - root.position), turned->joint - root.position) &&
                Near(*Rotate(lowerWorld, end.position - joint.position), turned->tip - turned->joint),
                "the reconstructed limb reaches the planted endpoint");

            const auto axis = *Unit(end.position - joint.position);
            const auto rolled = SolveEndpointRotation(limb, end.rotation, AxisAngle(axis, .65), joint.rotation, foot);
            Check(rolled && Near(rolled->joint, joint.position) && Near(rolled->tip, end.position),
                "an axial endpoint roll keeps the joints planted");
            Check(Length(*AngularCorrection(joint.rotation, rolled->lowerRotation, 1, 10)) > .1,
                "the forearm or shin absorbs part of the endpoint roll");

            auto current = end.rotation, following = joint.rotation;
            std::optional<Vec3> bend;
            bool reachedHingeLimit = false;
            const auto step = AxisAngle(axis, .025);
            for (int i = 0; i < 300; ++i) {
                const auto goal = Multiply(step, current);
                const auto edit = SolveEndpointRotation(limb, current, goal, following, foot, bend);
                current = goal;
                if (!edit) continue;
                Check(Near(edit->tip, end.position) && Near(*AngularCorrection(edit->endRotation, goal, 1, 10), {}),
                    "assisted endpoint reaches the requested orientation through a full turn");
                Check(Near(*AngularCorrection(Multiply(edit->lowerRotation, edit->localRotations[2]), goal, 1, 10), {}),
                    "the actual local rotation realizes the unrestricted endpoint goal");
                Check(Length(*AngularCorrection(following, edit->lowerRotation, 1, 10)) < .03,
                    "the upstream limb rotation stays continuous through a half-turn");
                following = edit->lowerRotation;
                bend = edit->bend;
                reachedHingeLimit |= edit->limited;
            }
            Check(reachedHingeLimit, "the endpoint continues turning after the upstream limb reaches its limit");

            // Near extension, a small turn must not throw the knee or elbow across the bend plane.
            auto extended = limb;
            const auto span = std::sqrt(1 - .05 * .05);
            extended.world[1].position = {0, .05, -span};
            extended.world[2].position = {0, 0, -2 * span};
            extended.hinge = CaptureLimbHinge(extended.world, {0, 1, 0}, foot);
            current = following = {};
            bend.reset();
            for (int i = 0; i < 25; ++i) {
                const auto goal = Multiply(AxisAngle({0, 1, 0}, .026), current);
                const auto edit = SolveEndpointRotation(extended, current, goal, following, foot, bend);
                current = goal;
                if (!edit) continue;
                Check(Length(*AngularCorrection(following, edit->lowerRotation, 1, 10)) < .07,
                    "the bend plane follows smoothly when the limb is almost straight");
                following = edit->lowerRotation;
                bend = edit->bend;
            }
        }
    }

    void WristTwist()
    {
        LimbPose arm{{Transform{{0, 0, 0}, {}}, Transform{{0, .6, -.8}, {}}, Transform{{0, 0, -1.6}, {}}},
            {}, {{}, {}, {}}, 1};
        arm.hinge = CaptureLimbHinge(arm.world, {0, 1, 0}, false);
        const Vec3 target{-.72, -.35, -1.18};
        const auto edit = SolveLimb(arm, target, arm.world[1].position, {}, {}, {}, false);
        Check(edit && Near(edit->tip, target), "automatic wrist orientation keeps the dragged hand at its target");
        const auto swing = *FromTo(arm.world[2].position - arm.world[1].position, edit->tip - edit->joint);
        const auto wristRoll = Length(*AngularCorrection(swing, edit->endRotation, 1, 10));
        const auto armRoll = Length(*AngularCorrection(swing, edit->lowerRotation, 1, 10));
        Check(armRoll > .05 && wristRoll < armRoll, "the arm follows the bend plane with less automatic roll at the wrist");

        const Vec3 shaft{0, 0, 1};
        const auto captured = AxisAngle({0, 1, 0}, .2);
        TwistPhase phase;
        SegmentTwistFrame saved;
        saved.proximal.angle = -1.65;
        saved.distal.angle = 3.31;
        // A saved pose provides the starting twist distribution for small edits.
        const auto savedJoint = AxisAngle({0, 1, 0}, 2.45);
        const auto savedHand = AxisAngle({1, 0, 0}, 1.91);
        Check(saved.Begin(savedJoint, {shaft * 16, savedHand}), "a restored pose supplies the twist reference");
        const auto [restProximal, restDistal] = saved.Measure(savedJoint, savedHand);
        Check(Near(restProximal, 0) && Near(restDistal, 0) && Near(saved.proximal.angle, -1.65) &&
            Near(saved.distal.angle, 3.31), "regrabbing a posed arm preserves helpers and saved winding");
        const auto [movedProximal, movedDistal] = saved.Measure(Multiply(savedJoint, AxisAngle(shaft, .01)), savedHand);
        Check(Near(movedProximal, .01) && Near(movedDistal, 0), "twist helpers follow only the new edit");
        const auto roll = phase.Update(AxisAngle(shaft, .75), shaft);
        const auto proximal = DistributeLimbTwist(captured, shaft, 0, roll, .29);
        const auto distal = DistributeLimbTwist(captured, shaft, 0, roll, .63);
        const auto proximalRoll = *AngularCorrection(captured, proximal, 1, 10);
        const auto distalRoll = *AngularCorrection(captured, distal, 1, 10);
        Check(Near(proximalRoll, shaft * (.29 * .75)) &&
            Near(distalRoll, shaft * (.63 * .75)), "helpers share roll progressively toward the endpoint");
        Check(Near(phase.Update(AxisAngle({1, 0, 0}, .8), shaft), 0), "a pure bend adds no axial roll");
        const auto first = phase.Update(AxisAngle(shaft, .75), shaft);
        Check(Near(phase.Update(AxisAngle({1, 0, 0}, std::numbers::pi), shaft), first),
            "a perpendicular half-turn holds the last defined twist");

        for (const double direction : {-1., 1.}) {
            TwistPhase winding;
            double previous = 0;
            for (int step = 1; step <= 720; ++step) {
                const auto angle = direction * step * std::numbers::pi / 180;
                auto sample = AxisAngle(shaft, angle);
                if (step % 2) sample = {-sample.x, -sample.y, -sample.z, -sample.w};
                const auto actual = winding.Update(sample, shaft);
                Check(std::abs(actual - previous) < .03, "helper roll is continuous across half-turns and quaternion sign changes");
                if (step % 360 == 180)
                    Check(Near(actual, direction * std::numbers::pi), "a half-turn retains full twist assistance");
                if (step % 360 == 0)
                    Check(Near(actual, 0), "a complete turn returns the skin helpers to their captured rotations");
                previous = actual;
            }
        }

        const auto shinAxis = *Unit(Vec3{1, 2, -3});
        const auto stance = AxisAngle(shinAxis, .6);
        const auto thigh = DistributeLimbTwist({}, shinAxis, .6, .4, .2);
        const auto nested = DistributeLimbTwist({}, shinAxis, .6, .4, .65);
        const auto nestedLocal = RelativeTo({{}, thigh}, {{}, nested});
        Check(nestedLocal && Near(*AngularCorrection({}, Multiply(stance, thigh), 1, 10), shinAxis * .2) &&
            Near(*AngularCorrection({}, Multiply(Multiply(stance, thigh), nestedLocal->rotation), 1, 10), shinAxis * .65),
            "rotations blend from the proximal joint toward the endpoint, including nested helpers");
    }

    void HierarchyReconstruction()
    {
        const Transform parent{{20, -7, 11}, *FromTo({1, 0, 0}, {.3, .8, .5})};
        const Transform upperLocal{{3, -1, .5}, *FromTo({1, 0, 0}, {.7, .4, .2})};
        const Transform lowerLocal{{2.3, 0, 0}, *FromTo({1, 0, 0}, {.5, -.4, .9})};
        const Transform tipLocal{{1.1, 0, 0}, {}};
        const auto originalUpper = Compose(parent, upperLocal);
        Check(originalUpper.has_value(), "hierarchy original upper exists");
        const auto originalLower = Compose(*originalUpper, lowerLocal);
        Check(originalLower.has_value(), "hierarchy original lower exists");
        const auto originalTip = Compose(*originalLower, tipLocal);
        Check(originalTip.has_value(), "hierarchy original tip exists");
        const auto target = originalUpper->position + *Rotate(parent.rotation, {1.7, .5, .9});
        const auto pole = originalUpper->position + *Rotate(parent.rotation, {.2, 3, 1.2});
        const auto solved = SolveTwoBone({originalUpper->position, originalLower->position,
            originalTip->position, target, pole, std::nullopt});
        Check(solved && !solved->targetClamped, "hierarchy unequal-length target reachable");

        const Transform desiredUpper{originalUpper->position,
            Multiply(solved->upperSwingWorld, originalUpper->rotation)};
        const Transform desiredLower{solved->joint,
            Multiply(solved->lowerSwingWorld, originalLower->rotation)};
        const auto editedUpperLocal = RelativeTo(parent, desiredUpper);
        Check(editedUpperLocal.has_value(), "hierarchy upper world-to-local succeeds");
        Check(Near(editedUpperLocal->position, upperLocal.position), "hierarchy upper attachment offset unchanged");
        const auto editedUpper = Compose(parent, {upperLocal.position, editedUpperLocal->rotation});
        Check(editedUpper.has_value(), "hierarchy reconstructed upper exists");

        // The lower bone must be relative to its newly rotated parent, not the captured parent.
        const auto editedLowerLocal = RelativeTo(*editedUpper, desiredLower);
        Check(editedLowerLocal.has_value(), "hierarchy lower world-to-local succeeds");
        Check(Near(editedLowerLocal->position, lowerLocal.position), "hierarchy elbow translation unchanged");
        const auto editedLower = Compose(*editedUpper, {lowerLocal.position, editedLowerLocal->rotation});
        Check(editedLower && Near(editedLower->position, solved->joint), "hierarchy forward kinematics reproduces IK elbow");
        const auto editedTip = Compose(*editedLower, tipLocal);
        Check(editedTip && Near(editedTip->position, solved->tip), "hierarchy forward kinematics reproduces IK tip");
        Check(Near(Length(editedLower->position - editedUpper->position), Length(lowerLocal.position)),
            "hierarchy upper segment retains original length");
        Check(Near(Length(editedTip->position - editedLower->position), Length(tipLocal.position)),
            "hierarchy lower segment retains original length");
        const auto recoveredTipLocal = RelativeTo(*editedLower, *editedTip);
        Check(recoveredTipLocal && Near(recoveredTipLocal->position, tipLocal.position), "hierarchy tip translation unchanged");

        const auto staleParentLocal = RelativeTo(*originalUpper, desiredLower);
        const auto incorrectLower = Compose(*editedUpper, {lowerLocal.position, staleParentLocal->rotation});
        const auto incorrectTip = Compose(*incorrectLower, tipLocal);
        Check(Length(incorrectTip->position - solved->tip) > .1, "hierarchy case detects stale-parent rotation conversion");
    }

    void TwistHierarchyReconstruction()
    {
        const Transform parent{{-13, 8, 24}, *FromTo({1, 0, 0}, {.2, .7, -.5})};
        const Transform upperLocal{{2, -1, .4}, *FromTo({1, 0, 0}, {.6, .3, .5})};
        const Transform forearmLocal{{2.6, 0, 0}, *FromTo({1, 0, 0}, {.4, -.5, .8})};
        const Transform twistOneLocal{{.3, 0, 0}, {std::sin(.32), 0, 0, std::cos(.32)}};
        const Transform twistTwoLocal{{.4, 0, 0}, {std::sin(-.19), 0, 0, std::cos(-.19)}};
        const Transform handLocal{{.7, 0, 0}, *FromTo({1, 0, 0}, {.7, .4, -.2})};
        const auto originalUpper = Compose(parent, upperLocal);
        Check(originalUpper.has_value(), "twist hierarchy captured upper exists");
        const auto originalForearm = Compose(*originalUpper, forearmLocal);
        Check(originalForearm.has_value(), "twist hierarchy captured forearm exists");
        const auto originalTwistOne = Compose(*originalForearm, twistOneLocal);
        Check(originalTwistOne.has_value(), "twist hierarchy first captured twist exists");
        const auto originalTwistTwo = Compose(*originalTwistOne, twistTwoLocal);
        Check(originalTwistTwo.has_value(), "twist hierarchy second captured twist exists");
        const auto originalHand = Compose(*originalTwistTwo, handLocal);
        Check(originalHand.has_value(), "twist hierarchy captured hand exists");
        const auto target = originalUpper->position + *Rotate(parent.rotation, {2.1, -.5, .3});
        const auto pole = originalUpper->position + *Rotate(parent.rotation, {.3, 1.6, 2.4});
        const auto solved = SolveTwoBone({originalUpper->position, originalForearm->position,
            originalHand->position, target, pole, std::nullopt});
        Check(solved && !solved->targetClamped, "twist hierarchy target is reachable");

        const Transform desiredUpper{originalUpper->position,
            Multiply(solved->upperSwingWorld, originalUpper->rotation)};
        const Transform desiredForearm{solved->joint,
            Multiply(solved->lowerSwingWorld, originalForearm->rotation)};
        const Transform desiredHand{solved->tip,
            Multiply(solved->lowerSwingWorld, originalHand->rotation)};
        const auto upperEdit = RelativeTo(parent, desiredUpper);
        Check(upperEdit && Near(upperEdit->position, upperLocal.position), "twist hierarchy upper attachment unchanged");
        const auto updatedUpper = Compose(parent, {upperLocal.position, upperEdit->rotation});
        Check(updatedUpper.has_value(), "twist hierarchy updated upper exists");
        const auto forearmEdit = RelativeTo(*updatedUpper, desiredForearm);
        Check(forearmEdit && Near(forearmEdit->position, forearmLocal.position), "twist hierarchy elbow attachment unchanged");
        const auto updatedForearm = Compose(*updatedUpper, {forearmLocal.position, forearmEdit->rotation});
        Check(updatedForearm && Near(updatedForearm->position, solved->joint), "twist hierarchy FK reproduces IK elbow");

        // Traverse both unchanged twist locals after applying the new forearm rotation.
        const auto updatedTwistOne = Compose(*updatedForearm, twistOneLocal);
        Check(updatedTwistOne.has_value(), "twist hierarchy updated first twist exists");
        const auto updatedTwistTwo = Compose(*updatedTwistOne, twistTwoLocal);
        Check(updatedTwistTwo.has_value(), "twist hierarchy updated second twist exists");
        const auto handEdit = RelativeTo(*updatedTwistTwo, desiredHand);
        Check(handEdit && Near(handEdit->position, handLocal.position), "twist hierarchy hand attachment unchanged");
        const auto updatedHand = Compose(*updatedTwistTwo, {handLocal.position, handEdit->rotation});
        Check(updatedHand && Near(updatedHand->position, solved->tip), "twist hierarchy FK reproduces IK hand endpoint");
        const Vec3 orientationAxis{0, 1, 0};
        Check(Near(*Rotate(updatedHand->rotation, orientationAxis), *Rotate(desiredHand.rotation, orientationAxis)),
            "twist hierarchy hand world orientation matches transported captured orientation");
        Check(Near(Length(updatedForearm->position - updatedUpper->position), 2.6), "twist hierarchy upper length unchanged");
        Check(Near(Length(updatedHand->position - updatedForearm->position), 1.4), "twist hierarchy lower length unchanged");
        const auto recoveredTwistOne = RelativeTo(*updatedForearm, *updatedTwistOne);
        const auto recoveredTwistTwo = RelativeTo(*updatedTwistOne, *updatedTwistTwo);
        Check(recoveredTwistOne && Near(recoveredTwistOne->position, twistOneLocal.position), "twist hierarchy first twist offset unchanged");
        Check(recoveredTwistTwo && Near(recoveredTwistTwo->position, twistTwoLocal.position), "twist hierarchy second twist offset unchanged");

        const auto wrongHandEdit = RelativeTo(*updatedForearm, desiredHand);
        Check(wrongHandEdit.has_value(), "twist hierarchy direct-parent negative control exists");
        const auto wrongHand = Compose(*updatedTwistTwo, {handLocal.position, wrongHandEdit->rotation});
        Check(wrongHand && Near(wrongHand->position, solved->tip), "twist hierarchy incorrect parent can conceal error in endpoint alone");
        Check(Length(*Rotate(wrongHand->rotation, orientationAxis) - *Rotate(desiredHand.rotation, orientationAxis)) > .1,
            "twist hierarchy detects forearm mistaken for actual hand parent");

        const LimbPose arm{{*originalUpper, *originalForearm, *originalHand}, parent.rotation,
            {upperLocal.rotation, forearmLocal.rotation, twistOneLocal.rotation, twistTwoLocal.rotation, handLocal.rotation}, 1};
        const auto reconstruct = [&](const LimbEdit& edit) {
            const auto a = Compose(parent, {upperLocal.position, edit.localRotations[0]});
            const auto b = Compose(*a, {forearmLocal.position, edit.localRotations[1]});
            const auto twist1 = Compose(*b, twistOneLocal);
            const auto twist2 = Compose(*twist1, twistTwoLocal);
            const auto c = Compose(*twist2, {handLocal.position, edit.localRotations[2]});
            Check(Near(a->position, originalUpper->position), "adapter keeps shoulder fixed");
            Check(Near(b->position, edit.joint), "adapter elbow matches IK");
            Check(Near(c->position, edit.tip), "adapter wrist matches IK through both twist bones");
            return *c;
        };
        const auto wristDrag = SolveLimb(arm, target, pole);
        Check(wristDrag && Near(wristDrag->tip, target), "wrist drag reaches the requested point");
        reconstruct(*wristDrag);
        const auto elbowDrag = SolveLimb(arm, originalHand->position, pole, {}, originalHand->rotation);
        Check(elbowDrag && Near(elbowDrag->tip, originalHand->position) &&
            Length(elbowDrag->joint - originalForearm->position) > 0.1, "pole drag changes elbow with fixed wrist position");
        const auto elbowDragWrist = reconstruct(*elbowDrag);
        Check(Near(*Rotate(elbowDragWrist.rotation, orientationAxis), *Rotate(originalHand->rotation, orientationAxis)),
            "pole drag preserves wrist orientation");
        const auto limbAxis = *Unit(originalHand->position - originalUpper->position);
        const auto planeTurn = AxisAngle(limbAxis, .42);
        const auto turnedElbow = originalUpper->position +
            *Rotate(planeTurn, originalForearm->position - originalUpper->position);
        for (std::size_t i = 0; i < 2; ++i) {
            const auto axis = *Unit(arm.world[i + 1].position - arm.world[i].position);
            const auto roll = Multiply(AxisAngle(axis, .38), arm.world[i].rotation);
            const auto edit = SolveLimb(arm, originalHand->position, arm.world[1].position, {},
                originalHand->rotation, LimbRotation{i, roll});
            Check(edit && Near(edit->joint, originalForearm->position) &&
                Near(edit->tip, originalHand->position), "rolling either limb segment keeps both joints planted");
            const auto wrist = reconstruct(*edit);
            Check(Near(*Rotate(wrist.rotation, orientationAxis), *Rotate(originalHand->rotation, orientationAxis)),
                "the planted wrist retains its world rotation while either segment rolls");
            Check(std::abs(Dot(Vec3{edit->localRotations[i].x, edit->localRotations[i].y, edit->localRotations[i].z},
                Vec3{arm.localRotations[i].x, arm.localRotations[i].y, arm.localRotations[i].z}) +
                edit->localRotations[i].w * arm.localRotations[i].w) < .995, "the requested segment actually rolls");
            const auto swing = SolveLimb(arm, originalHand->position, arm.world[1].position, {},
                originalHand->rotation, LimbRotation{i, Multiply(planeTurn, arm.world[i].rotation)});
            Check(swing && Near(swing->joint, turnedElbow) && Near(swing->tip, originalHand->position),
                "turning either segment can rotate the bend plane around the planted wrist");
            reconstruct(*swing);
        }
    }

    void ReplicatedPoseTranslation()
    {
        const auto parent = AxisAngle({0, 0, 1}, std::numbers::pi / 2);
        const Vec3 originalLocal{2, 5, -1}, movement{8, -12, 6};
        const auto delta = LocalTranslationOffset(movement, parent, 2);
        Check(delta && Near(*delta, {-6, -4, 3}), "COM translation uses inverse parent rotation and scale");
        const auto originalWorld = *Rotate(parent, originalLocal * 2);
        const auto movedWorld = *Rotate(parent, (originalLocal + *delta) * 2);
        Check(Near(movedWorld - originalWorld, movement), "a replicated COM offset preserves horizontal and vertical pose movement");
        // After landing, encode the former airborne offset in the replicated local transform.
        const auto jumpHeight = Vec3{0, 0, 48};
        const auto groundedPosition = Vec3{100, 20, 150};
        const auto target = groundedPosition + jumpHeight + movedWorld;
        const auto groundedLocal = LocalTranslationOffset(target - groundedPosition, parent, 2);
        Check(groundedLocal && Near(groundedPosition + *Rotate(parent, *groundedLocal * 2), target),
            "COM rebased to the grounded gameplay actor preserves the frozen jump height for the peer");
        // The furniture exit can translate and turn the gameplay actor.
        const Transform seatFrame{{20, 40, 15}, parent};
        const Transform standingFrame{{110, 44, 0}, AxisAngle({0, 0, 1}, -.55)};
        const auto seatedCom = *Compose(seatFrame, {originalLocal, parent});
        const auto restoredSeat = *Compose(standingFrame, *RelativeTo(standingFrame, seatedCom));
        Check(Near(restoredSeat.position, seatedCom.position) &&
            Near(*Rotate(restoredSeat.rotation, {0, 1, 0}), *Rotate(seatedCom.rotation, {0, 1, 0})),
            "a seated pose preserves the world anchor after the furniture exit moves and turns the actor");
        Check(!LocalTranslationOffset(movement, parent, 0), "degenerate parent scale rejects a translation");
        const Vec3 lineOfSight{.3, .2, .8};
        constexpr double strength = 1.0 / 30;
        // The refresh offset must cross PlayerHub’s quantization boundary.
        const auto refresh = PoseRefreshOffset({1.028, -2, 3}, lineOfSight, strength);
        Check(Near(Cross(refresh, lineOfSight), {}) && Length(refresh) <= strength,
            "a pose refresh crosses the closest rounding boundary along the viewing line");
        Check(std::round(float(1.028 + refresh.x) * 16) != std::round(float(1.028) * 16),
            "the refresh survives conversion to the engine's float");
        const auto diagonalRefresh = PoseRefreshOffset({}, {1, 1, 1}, strength);
        Check(Length(diagonalRefresh) <= strength && std::round(float(diagonalRefresh.x) * 16) != 0,
            "a diagonal view uses one axis when the camera-aligned nudge would exceed the cap");
        Check(Near(PoseRefreshOffset({}, lineOfSight, 0), {}), "zero refresh strength leaves the pose still");
    }

    void CameraLook()
    {
        const Vec3 forward{0, 1, 0}, right{1, 0, 0};
        const auto turn = CameraLookDelta(forward, right, -std::numbers::pi / 2, std::numbers::pi / 4);
        const auto aim = Rotate(turn, forward);
        const auto across = Rotate(turn, right);
        const auto diagonal = std::sqrt(0.5);
        Check(aim && Near(*aim, {diagonal, 0, diagonal}), "camera yaws to the right and pitches up around its new right");
        Check(across && Near(*across, {0, -1, 0}), "successive yaw and pitch keep the camera level");
        const auto atLimit = Rotate(CameraLookDelta(forward, right, 0, 9), forward);
        Check(atLimit && atLimit->z > 0.999 && atLimit->y > 0, "pitch stops before the camera flips");
    }

    void CameraGazeAndMirror()
    {
        using namespace FacePose;
        const Vec3 right{1, 0, 0}, forward{0, 1, 0}, up{0, 0, 1};
        const auto straight = AimGaze(forward, right, forward, up);
        Check(straight && straight->weights[LookLeft] == 0 && straight->weights[LookRight] == 0 &&
            straight->weights[LookUp] == 0 && straight->weights[LookDown] == 0, "centered gaze clears all four direction sliders");
        const auto diagonal = AimGaze({-1, 3, 1}, right, forward, up);
        Check(diagonal && diagonal->weights[LookLeft] > 0 && diagonal->weights[LookUp] > 0 &&
            diagonal->weights[LookRight] == 0 && diagonal->weights[LookDown] == 0, "camera gaze resolves into two opposing channel pairs");
        const auto turned = AxisAngle(up, std::numbers::pi / 2);
        const auto relative = AimGaze(forward, *Rotate(turned, right), *Rotate(turned, forward), up);
        Check(relative && relative->weights[LookRight] == 1 && !AimGaze({}, right, forward, up),
            "gaze is head-relative, clamps to the slider range, and ignores a coincident camera");

        const BoneMirrorFrame left{AxisAngle(forward, .7), AxisAngle(up, -.4)};
        const BoneMirrorFrame opposite{AxisAngle(right, -.3), AxisAngle(forward, .2)};
        // Mirroring must preserve each side’s own bind axes before reflecting an edit.
        const auto matchingRest = MirrorBoneRotation(left.local, left, opposite, right);
        Check(Near(*AngularCorrection(opposite.local, matchingRest, 1, 10), {}),
            "mirroring the left rest pose resolves to the right bone's own rest orientation");
        const auto source = Multiply(AxisAngle(forward, .45), left.local);
        const auto target = MirrorBoneRotation(source, left, opposite, right);
        const auto restored = MirrorBoneRotation(target, opposite, left, right);
        Check(Near(*AngularCorrection(source, restored, 1, 10), {}),
            "the same mirrored joint pose round-trips in either direction");
        const auto a = Multiply(Multiply(left.parent, Multiply(source, Fingers::Conjugate(left.local))),
            Fingers::Conjugate(left.parent));
        const auto b = Multiply(Multiply(opposite.parent, Multiply(target, Fingers::Conjugate(opposite.local))),
            Fingers::Conjugate(opposite.parent));
        const auto reflect = [right](Vec3 value) { return value - right * (2 * Dot(value, right)); };
        const Vec3 probe{.4, -.2, .7};
        Check(Near(*Rotate(b, reflect(probe)), reflect(*Rotate(a, probe))),
            "mirrored skeletal rotation produces the reflected world motion even with different paired parents");
        const auto finger = Fingers::FromAngles({.6, -.25, .35});
        const auto leftLocal = Fingers::ToLocal(left.local, finger, Fingers::Side::Left);
        const auto rightLocal = Fingers::ToLocal(opposite.local, finger, Fingers::Side::Right);
        Check(Near(*AngularCorrection(Fingers::FromLocal(left.local, leftLocal, Fingers::Side::Left),
            Fingers::FromLocal(opposite.local, rightLocal, Fingers::Side::Right), 1, 10), {}),
            "paired finger edits share canonical curl, spread, and mirrored twist");
    }

    void PoseHistoryActions()
    {
        using enum HistoryAction;
        PoseHistory<int> history;
        history.Commit(0);
        history.Commit(1);
        history.Commit(1);
        history.Commit(2);
        Check(history.States() == std::vector{0, 1, 2}, "release checkpoints skip repeated values");
        Check(*history.Move(Undo) == 1 && *history.Move(Undo) == 0 && !history.Move(Undo),
            "undo walks completed actions back to the opening state");
        Check(*history.Move(Redo) == 1, "redo advances one action");
        // Reopening on the same pose keeps redo. Reopening on a different pose adds an edit.
        PoseHistory<int> reopened;
        Check(reopened.Restore(history.States(), history.Cursor()) && !reopened.Dirty(),
            "a saved history remembers both checkpoints and cursor");
        reopened.Commit(1);
        Check(reopened.CanMove(Redo) && !reopened.Dirty(), "reopening the same pose preserves the redo branch");
        reopened.Commit(9);
        Check(*reopened.Move(Undo) == 1 && *reopened.Move(Redo) == 9 && reopened.States() == std::vector{0, 1, 9},
            "a changed opening pose is an ordinary undo and redo checkpoint");
        reopened.Commit(8);
        reopened.Commit(10);
        Check(*reopened.Move(Undo) == 8 && *reopened.Move(Undo) == 9,
            "editing after reopening preserves the new starting pose");
        reopened.Commit(3);
        Check(!reopened.CanMove(Redo), "editing after undo discards the old redo branch");
        for (int i = 11; i < 99; ++i) reopened.Commit(i);
        Check(reopened.States().size() == PoseHistory<int>::limit && reopened.States().back() == 98,
            "history retains the newest checkpoints at its storage limit");
        Check(!reopened.Restore({1, 2}, 2) && reopened.States().back() == 98,
            "an invalid stored cursor cannot replace the current history");
        // Legacy history can contain duplicate neighbors that share an undo position.
        PoseHistory<int> repeated;
        Check(repeated.Restore({1, 1, 2, 2, 1, 1}, 3) && repeated.States() == std::vector{1, 2, 1} &&
            repeated.Cursor() == 1 && repeated.Dirty(), "loading collapses adjacent duplicates and keeps repeated poses separated by edits");
    }

    void SharedPoseHistory()
    {
        struct Scene
        {
            int body{}, left{}, right{}, face{};
            bool operator==(const Scene&) const = default;
        };
        using enum HistoryAction;
        // Each editor contributes to one undo sequence in edit order.
        PoseHistory<Scene> history;
        history.Commit({});
        history.Commit({1, 0, 0, 0});
        history.Commit({1, 2, 0, 0});
        history.Commit({1, 2, 0, 3});
        history.Commit({1, 2, 4, 3});
        Check(*history.Move(Undo) == Scene{1, 2, 0, 3} && *history.Move(Undo) == Scene{1, 2, 0, 0} &&
            *history.Move(Undo) == Scene{1, 0, 0, 0} && *history.Move(Undo) == Scene{},
            "undo replays body, left hand, face and right hand edits in reverse chronological order");
        Check(history.Move(Redo) && history.Move(Redo), "redo can step forward across editor types");
        PoseHistory<Scene> reopened;
        Check(reopened.Restore(history.States(), history.Cursor()), "reopening preserves the shared history cursor");
        reopened.Commit({1, 2, 0, 0});
        Check(*reopened.Move(Redo) == Scene{1, 2, 0, 3} && *reopened.Move(Redo) == Scene{1, 2, 4, 3},
            "reopening the same pose retains cross-editor redo");
        reopened.Commit({7, 8, 9, 10});
        reopened.Commit({7, 8, 9, 10});
        Check(*reopened.Move(Undo) == Scene{1, 2, 4, 3} && *reopened.Move(Redo) == Scene{7, 8, 9, 10},
            "reopening a different whole pose adds one undoable and redoable checkpoint");
        // A whole twist turn changes the winding even when the displayed angle wraps.
        using Angles = PoseHistory<double, decltype([](double left, double right) {
            return TwistPhase::SameAngle(left, right);
        })>;
        Angles phases;
        phases.Commit(.2);
        phases.Commit(.2 + 2 * std::numbers::pi);
        phases.Commit(.2 + 2 * std::numbers::pi + 1e-8);
        Check(phases.States().size() == 2 && Near(*phases.Move(Undo), .2),
            "history preserves twist winding while ignoring serialization noise");
    }

    const auto bundledPoses = std::filesystem::path(__FILE__).parent_path().parent_path() /
        "assets/SKSE/Plugins/GModPose/Presets/Premade";

    const auto& HandPresets()
    {
        static const auto presets = PoseSerialization::ReadFolder<PoseSerialization::HandPreset>(
            bundledPoses / "Hand", "hand", 128, [](const auto& object, auto& entry) {
                return entry.category == PosePresetJson::Category::Premade && PoseSerialization::DecodeHand(object, entry.pose);
            });
        return presets;
    }

    void TestPresetFiles()
    {
        using namespace PoseSerialization;
        for (const std::string_view character : {"A", "\xC3\xA9", "\xE5\xBA\xA7", "\xF0\x9F\x98\x80"}) {
            std::string name;
            for (std::size_t i = 0; i < maxNameCharacters; ++i) name += character;
            Check(ValidName(name) && name.size() < nameBufferSize, "64 Unicode characters fit the preset field");
            std::string readName;
            Category category{};
            std::istringstream encoded(Document("face", name, Category::Custom, Json::object()).dump());
            Check(Header(Read(encoded), "face", readName, category) && readName == name,
                "a 64-character name survives saving and reopening JSON");
            name += character;
            Check(!ValidName(name), "the name limit counts Unicode characters");
        }
        Check(!ValidName("") && !ValidName("name\n") && !ValidName("\x7F") && !ValidName("\xC3") &&
            !ValidName("\xE0\x80\xAF") && !ValidName("\xED\xA0\x80") && !ValidName("\xF4\x90\x80\x80"),
            "invalid UTF-8 and control characters are rejected in names");

        const auto faces = ReadFolder<FacePreset>(bundledPoses / "Face", "face", 128,
            [](const auto& object, auto& entry) {
                return entry.category == Category::Premade && DecodeFacePose(object, entry.pose);
            });
        Check(faces.size() == 19 && HandPresets().size() == 25, "all bundled face and hand presets load");
        const auto cheese = std::ranges::find(faces, "Cheese", &FacePreset::name);
        const auto smile = std::ranges::find(faces, "Smile", &FacePreset::name);
        Check(cheese != faces.end() && smile != faces.end() &&
            cheese->pose.mask == FacePose::all && cheese->pose.pose.mood == FacePose::Expression(3).mood &&
            cheese->pose.pose.weights[FacePose::Eee] > smile->pose.pose.weights[FacePose::Eee] &&
            cheese->pose.pose.weights[FacePose::Aah] > 0,
            "Cheese retains the wider, slightly open smile");
        // A missing face channel stays unchanged, while an explicit zero clears it.
        std::istringstream sparse(R"({"schema":1,"type":"face","name":"Wink","pose":{"weights":{"blink_left":1,"blink_right":0}}})");
        const auto document = Read(sparse);
        FacePreset preset;
        Check(Header(document, "face", preset.name, preset.category) && preset.category == Category::Custom &&
            DecodeFacePose(Field(document, "pose"), preset.pose) && preset.pose.mask == FacePose::blink,
            "sparse face preset touches only the authored eyelids");
        auto face = FacePose::Expression(3);
        face.weights[FacePose::BlinkRight] = .6F;
        FacePose::Merge(face, preset.pose.pose, preset.pose.mask);
        Check(face == FacePose::State{FacePose::With(FacePose::BlinkLeft, 1), FacePose::neutralMood + 3, .8F},
            "an explicit zero clears a channel while an omitted mood stays in place");
        Check(EncodeFacePose(preset.pose) == Field(document, "pose"), "sparse face round trips");
        const auto& hand = HandPresets().front();
        Fingers::Pose handResult;
        Check(DecodeHand(EncodeHand(hand.pose), handResult) &&
            std::ranges::equal(hand.pose, handResult, [](Quaternion a, Quaternion b) {
                return Near(a.x, b.x) && Near(a.y, b.y) && Near(a.z, b.z) && Near(a.w, b.w);
            }), "hand rotations round trip");
        // Legacy snapshots carry a full expression and must retain explicit zeroes.
        std::ostringstream legacyRow;
        legacyRow << "10 0.75";
        for (unsigned i = 0; i < FacePose::weightCount; ++i) legacyRow << (i == 0 ? " 1" : " 0");
        std::istringstream legacyFace(legacyRow.str());
        Check(DecodeLegacyFace(legacyFace, preset.pose) && preset.pose.mask == FacePose::all &&
            EncodeFacePose(preset.pose)["weights"].size() == FacePose::weightCount,
            "legacy face snapshots retain all explicit weights");

        struct Frame {
            struct Bone { Transform local; float scale; std::optional<Transform> root; float rootScale; bool manual; };
            Vec3 translation;
            std::vector<Bone> bones;
            std::array<double, 2> twistPhases{};
            std::optional<FaceActor::Sample> face;
        };
        Frame old{{1, -2, 3}, {{{{4, 5, 6}, {}}, 1, Transform{{1, 2, 3}, {}}, .5F, true}},
            {3, -7}, FaceActor::Sample{preset.pose.pose, FacePose::all}};
        // Body transforms are indexed by rig, including its root layout and bone count.
        constexpr auto rig = "18386927322907410144";
        const auto object = EncodeBody(old, rig);
        std::istringstream bodyJson(object.dump());
        Frame current;
        Check(DecodeBody(Read(bodyJson), current, rig, 1, [](auto) { return true; }) &&
            EncodeBody(current, rig) == object, "body snapshot round trips with face, root and twist");
        Check(!DecodeBody(object, current, "another", 1, [](auto) { return true; }) &&
            !DecodeBody(object, current, rig, 1, [](auto) { return false; }) &&
            !DecodeBody(object, current, rig, 0, [](auto) { return true; }),
            "body preset rejects a different rig, root layout or bone count");
    }

    void FingerGestures()
    {
        using namespace GModPose::Fingers;
        const auto rest = AxisAngle({0, 0, 1}, 0.28);
        const auto canonical = FromAngles({0.71, 0.26, -0.17});
        for (auto side : {Side::Left, Side::Right}) {
            const auto local = ToLocal(rest, canonical, side);
            const auto recovered = FromLocal(rest, local, side);
            Check(Near(Vec3{recovered.x, recovered.y, recovered.z}, {canonical.x, canonical.y, canonical.z}) &&
                Near(recovered.w, canonical.w), "finger local rotation round trips for each hand");
        }
        const auto angles = ReadAngles(canonical);
        Check(Near(angles.curl, 0.71) && Near(angles.spread, 0.26) && Near(angles.roll, -0.17),
            "finger drag decomposes all three local rotations");
        const auto& presets = HandPresets();
        Check(std::ranges::is_sorted(presets, {}, &PoseSerialization::HandPreset::name), "premade gestures appear in alphabetical order");
        for (const auto& preset : presets) {
            for (const auto& q : preset.pose) {
                const auto norm = Unit(q);
                Check(norm && Near(q.w, norm->w) && Near(Vec3{q.x, q.y, q.z}, {norm->x, norm->y, norm->z}),
                    "each preset has 15 finite unit rotations");
            }
        }
        const auto curl = [&](std::string_view name, std::size_t digit) {
            const auto match = std::ranges::find(presets, name, &PoseSerialization::HandPreset::name);
            return match == presets.end() ? 99.0 : ReadAngles(match->pose[digit * jointsPerDigit + 1]).curl;
        };
        Check(curl("Point", 1) < 0 && curl("Point", 2) > 0.8, "point extends the index finger");
        Check(curl("Pistol", 0) < 0 && curl("Pistol", 1) < 0 && curl("Pistol", 4) > 0.8,
            "pistol extends the thumb and index finger");
        Check(curl("Prayer", 1) < 0 && curl("Prayer", 4) < 0, "prayer straightens the long fingers");
        Check(curl("Heart", 0) < 0, "the hand heart extends the thumb");
        for (std::size_t digit = 1; digit < digitCount; ++digit)
            Check(curl("Heart", digit) > 0 && curl("Heart", digit) < curl("Fist", digit),
                "all four long fingers arch in the hand heart");
        Check(curl("Fingers crossed", 1) < 0 && curl("Fingers crossed", 2) < 0 &&
            curl("Fingers crossed", 3) > 0.8, "crossed fingers extend the index and middle");
        Check(curl("Spock", 1) < 0 && curl("Spock", 4) < 0,
            "the split-hand salute extends both finger pairs");
        Check(curl("Stop", 1) < 0 && curl("Stop", 4) < 0, "the stop-hand preset extends the fingers");
        Check(curl("Holding", 1) > 0 && curl("Holding", 1) < curl("Claw", 1),
            "the holding preset is rounder than a claw");
        Check(curl("Claw", 1) > 0.4 && curl("Claw", 1) < curl("Fist", 1), "claw curls less tightly than a fist");
        Check(curl("Pinky", 0) > 0 && curl("Pinky", 1) > 0.8 && curl("Pinky", 4) < 0, "pinky extends only the little finger");
        Check(curl("Call me", 0) < 0 && curl("Call me", 4) < 0 && curl("Call me", 1) > 0.8,
            "call me extends the thumb and little finger");
        Check(curl("Rock horns", 1) < 0 && curl("Rock horns", 4) < 0 &&
            curl("Rock horns", 2) > 0.8, "rock horns extend the index and little finger");
        const std::array<std::string_view, 5> countNames{"Count 1", "Count 2", "Count 3", "Count 4", "Count 5"};
        for (std::size_t i = 0; i < countNames.size(); ++i) {
            unsigned extended = 0;
            for (std::size_t digit = 0; digit < digitCount; ++digit) extended += curl(countNames[i], digit) < 0;
            Check(extended == i + 1, "each counting preset extends the stated number of digits");
        }
    }

    void FingerDiagramDrag()
    {
        using namespace Fingers;
        // Rotate the diagram segment to test that curl and spread follow its local axes.
        for (const double heading : {-1.1, -.25, .4}) {
            const DiagramSegment segment{{std::sin(heading), -std::cos(heading), 0}, 30};
            const Angles start{.25, .3, -.7};
            const auto startTip = segment.Tip(start);
            const auto curled = segment.Drag(startTip * .8, start);
            Check(curled.curl > start.curl && Near(curled.spread, start.spread) && Near(curled.roll, start.roll),
                "pulling along an already spread finger curls without changing spread or twist");
            for (const double mirror : {-1., 1.}) {
                const Vec3 goal = segment.Tip({-.2, -.4});
                const Vec3 screenMove{mirror * (goal.x - startTip.x), goal.y - startTip.y, 0};
                const auto edited = segment.Drag(startTip + Vec3{mirror * screenMove.x, screenMove.y, 0}, start);
                Check(Near(segment.Tip(edited), goal) && Near(edited.roll, start.roll),
                    "the finger triangle tracks a diagonal drag on either hand");
            }
            const auto pastPalm = segment.Drag(startTip * -.1, start);
            Check(Near(pastPalm.curl, 1.52) && Near(pastPalm.spread, start.spread),
                "dragging past the pivot saturates curl without reversing the finger");
        }
    }

    void FingerGestureGeometry()
    {
        using namespace GModPose::Fingers;
        // Skyrim's right-hand reference transforms, relative to the parent.
        constexpr std::array<std::array<Transform, 3>, 3> rest{{
            {{{{2.86040,-.92021,3.16704},{.060519,.390356,-.531396,.749385}},
              {{.00001,-.00001,3.23610},{.164966,-.077377,.012983,.983174}},
              {{0,0,2.83698},{.043645,-.024409,.001067,.998748}}}},
            {{{{2.54483,-.10912,8.39495},{.229097,.009852,.055827,.971751}},
              {{0,0,2.45817},{.133016,0,0,.991114}},
              {{0,0,1.89273},{.128123,0,0,.991758}}}},
            {{{{.36776,-.08281,8.26200},{.251772,-.015089,.092505,.963237}},
              {{0,0,3.20164},{.146967,0,0,.989141}},
              {{0,0,1.98943},{.146893,0,0,.989152}}}},
        }};
        const auto chain = [&](const Pose& pose, unsigned digit, double length) {
            std::array<Vec3, 4> points;
            Transform transform{};
            for (unsigned i = 0; i < jointsPerDigit; ++i) {
                auto local = rest[digit][i];
                local.rotation = ToLocal(*Unit(local.rotation), pose[digit * jointsPerDigit + i], Side::Right);
                transform = *Compose(transform, local);
                points[i] = transform.position;
            }
            points.back() = transform.position + *Rotate(transform.rotation, {0, 0, length});
            return points;
        };
        const auto lastSegment = [&](const Pose& pose, unsigned digit, double length) {
            const auto points = chain(pose, digit, length);
            return ScreenSegment{points[2], points[3]};
        };
        const auto& presets = HandPresets();
        const auto heart = std::ranges::find(presets, "Heart", &PoseSerialization::HandPreset::name);
        const auto fist = std::ranges::find(presets, "Fist", &PoseSerialization::HandPreset::name);
        Check(heart != presets.end() && fist != presets.end(), "geometry presets exist");
        Check(std::ranges::all_of(heart->pose, [](Quaternion joint) { return Near(ReadAngles(joint).roll, 0); }),
            "the Heart preset has no axial twist");
        const auto thumb = lastSegment(heart->pose, 0, 2.0);
        const auto index = lastSegment(heart->pose, 1, 1.9);
        const auto gap = index.end - thumb.end;
        Check(thumb.end.x > 6 && thumb.end.x > thumb.start.x + .8,
            "the Test10 Heart thumb angles outward from its distal joint");
        Check(index.end.x > index.start.x + .6 && index.end.z < index.start.z,
            "the Heart index turns inward and down at the end of the arch");
        Check(gap.z > 5 && gap.y > 4 && Length(gap) > 7,
            "the Test10 Heart leaves its distinct opening between the thumb and index");
        const auto fistThumb = lastSegment(fist->pose, 0, 2.0).end;
        Check(fistThumb.y < -5.5 && fistThumb.x > 1.2 && fistThumb.x < 2.5,
            "the fist thumb rests in front of the folded fingers");
        const auto ok = std::ranges::find(presets, "OK", &PoseSerialization::HandPreset::name);
        const auto pinch = Length(lastSegment(ok->pose, 0, 2.0).end - lastSegment(ok->pose, 1, 1.9).end);
        const auto open = std::ranges::find(presets, "Open hand", &PoseSerialization::HandPreset::name);
        const auto openGap = Length(lastSegment(open->pose, 0, 2.0).end - lastSegment(open->pose, 1, 1.9).end);
        Check(pinch < openGap * .2, "the saved OK pose brings the thumb and index tips close on the reference rig");
        const auto thumbsUp = std::ranges::find(presets, "Thumbs up", &PoseSerialization::HandPreset::name);
        const auto raisedThumb = chain(thumbsUp->pose, 0, 2.0);
        const auto thumbDirection = raisedThumb.back() - raisedThumb.front();
        Check(thumbDirection.x > 2 * std::hypot(thumbDirection.y, thumbDirection.z),
            "thumbs up extends mainly above the fist on the reference rig");
        const auto holding = std::ranges::find(presets, "Holding", &PoseSerialization::HandPreset::name);
        Check(lastSegment(holding->pose, 0, 2.0).end.x > 5,
            "the holding thumb stays opposed to the fingers");
        // Check the crossing in the palm projection and the depth clearance in 3D.
        const auto crossed = std::ranges::find(presets, "Fingers crossed", &PoseSerialization::HandPreset::name);
        const auto front = chain(crossed->pose, 1, 1.9), rear = chain(crossed->pose, 2, 1.85);
        const auto a = front[2] - front[1], b = rear[2] - rear[1], gapAtKnuckles = rear[1] - front[1];
        const auto crossInPalm = [](Vec3 u, Vec3 v) { return u.x * v.z - u.z * v.x; };
        const auto det = crossInPalm(a, b);
        const auto t = crossInPalm(gapAtKnuckles, b) / det, u = crossInPalm(gapAtKnuckles, a) / det;
        Check(t > 0 && t < 1 && u > 0 && u < 1 && (front[1] + a * t - rear[1] - b * u).y > 1.6,
            "crossed fingers overlap in outline but have depth clearance at the crossover");
    }

    void WholePoseRotation()
    {
        const Vec3 center{10, -8, 15};
        const Transform wrist{center + Vec3{3, 4, 6}, AxisAngle({1, 0, 0}, 0.4)};
        const auto yaw = AxisAngle({0, 0, 1}, std::numbers::pi / 2);
        const auto turned = RotateAround(wrist, center, yaw);
        Check(turned && Near(turned->position, center + Vec3{-4, 3, 6}), "whole-pose turn follows world Z");
        const auto turnedCenter = RotateAround({center, {}}, center, yaw);
        Check(turnedCenter && Near(turnedCenter->position, center), "whole-pose turn leaves the COM pivot fixed");
        const auto local = RelativeTo({center, {}}, wrist);
        const auto restoredLocal = RelativeTo(*turnedCenter, *turned);
        Check(local && restoredLocal && Near(local->position, restoredLocal->position) &&
            Near(*AngularCorrection(local->rotation, restoredLocal->rotation, 1, 10), {}),
            "whole-pose turn preserves the offset and rotation between the COM and the wrist");
    }

    void IdleBreathing()
    {
        BreathingCycle breath;
        Check(breath.Update(false, .05) == 0, "disabled breathing is still");
        const auto first = breath.Update(true, .05);
        Check(first > 0 && first < .001, "breathing eases in from the resting pose");
        double lift = first;
        for (int frame = 1; frame < 45; ++frame) lift = breath.Update(true, .05);
        Check(Near(lift, .20), "the chest reaches its lift limit halfway through a breath");
        Check(breath.Update(true, std::numeric_limits<double>::quiet_NaN()) == lift,
            "a bad frame delta leaves the breath unchanged");
        for (int frame = 45; frame < 90; ++frame) lift = breath.Update(true, .05);
        Check(Near(lift, 0), "the chest returns to rest at the end of the breath");
        for (int frame = 0; frame < 45; ++frame) lift = breath.Update(true, .05);
        // Enter and leave an edit at peak lift to catch a discontinuous fade or restart.
        const auto fading = breath.Update(false, .05);
        Check(fading > 0 && fading < lift, "editing fades the chest motion out");
        for (int frame = 0; frame < 5; ++frame) lift = breath.Update(false, .05);
        Check(lift == 0 && breath.Update(false, .05) == 0, "breathing stays at rest during edits");
        Check(Near(breath.Update(true, .05), first), "breathing resumes from rest after an edit");
    }

    void PlantedEndpoints()
    {
        // Move each bent chain's root and solve back to its endpoint anchor.
        for (const Vec3 hip : {Vec3{-3,0,0}, Vec3{3,0,0}, Vec3{-3,0,7}, Vec3{3,0,7}}) {
            const Quaternion endpointRotation = AxisAngle({0, 0, 1}, 0.3);
            const Transform anchor{hip + Vec3{0, 1, -6}, endpointRotation};
            LimbPose shifted{{Transform{hip, {}}, Transform{hip + Vec3{0, 2, -3}, {}}, anchor}, {}, {{}, {}, endpointRotation}, 1};
            for (auto& joint : shifted.world) joint.position = joint.position + Vec3{0.8, 0.4, 0};
            const auto edit = SolveAnchoredLimb(shifted, anchor);
            Check(edit && Near(edit->tip, anchor.position), "a shifted chain keeps its endpoint planted");
            Check(Length(edit->joint - shifted.world[1].position) > 0.01, "planted endpoint allows the intermediate joint to follow");
            Check(Near(*AngularCorrection(edit->endRotation, endpointRotation, 1, 10), {}), "planted endpoint retains its orientation");
            for (auto& joint : shifted.world) joint.position.x += 20;
            Check(!SolveAnchoredLimb(shifted, anchor), "a shift beyond limb reach is rejected");
        }
    }

    void DraggingAndPicking()
    {
        const Ray initial{{0, 0, 0}, {0, 0, 1}};
        const auto drag = BeginDrag(initial, {1, 2, 10}, {0, 0, 1});
        Check(drag && Near(*DragTarget(*drag, initial), {1, 2, 10}), "off-center grab does not snap");
        Check(Near(*DragTarget(*drag, {{0, 0, 0}, {.1, .2, 1}}), {2, 4, 10}), "screen plane movement");
        Check(Near(*DragTarget(*drag, initial, 5), {1, 2, 15}), "wheel depth moves captured plane");
        Check(Near(*DragTarget(*drag, {{2, 3, 0}, {0, 0, 1}}), {3, 5, 10}), "camera movement retains captured plane");
        Check(!DragTarget(*drag, initial, -20), "behind-camera depth rejected");
        Check(!DragTarget(*drag, {{}, {1, 0, 0}}), "parallel ray rejected");
        Check(!BeginDrag(initial, {0, 0, 10}, {}), "zero plane normal rejected");
        const std::array<ScreenHandle, 4> handles{{{1, 10, 10, 100}, {2, 10, 10, 50}, {3, 11, 10, -1}, {4, 30, 30, 10}}};
        Check(PickHandle(handles, 10, 10, 8) == 2, "overlapping handles prefer nearer visible depth");
        Check(!PickHandle(handles, 100, 100, 8), "miss selects nothing");
        Check(PickHandle(handles, 38, 30, 8) == 4, "pick radius boundary included");
        Check(!PickHandle(handles, 10, 10, -1), "negative radius rejected");
    }

    void InvalidNumbers()
    {
        // Bad math inputs must fail before invalid transforms can reach the skeleton.
        const auto nan = std::numeric_limits<double>::quiet_NaN();
        const auto infinity = std::numeric_limits<double>::infinity();
        auto arm = Arm(); arm.target.x = nan;
        Check(!SolveTwoBone(arm), "NaN IK input rejected");
        arm = Arm(); arm.previousBend = Vec3{infinity, 0, 0};
        Check(!SolveTwoBone(arm), "nonfinite fallback rejected");
        Check(!IntersectPlane({{}, {0, 0, 1}}, {0, 0, infinity}, {0, 0, 1}), "nonfinite plane rejected");
        Check(!Rotate({nan, 0, 0, 1}, {1, 0, 0}), "NaN quaternion rejected");
        Check(!Compose({{infinity, 0, 0}, {}}, {}), "nonfinite parent rejected");
        Check(!Unit(Vec3{infinity, 0, 0}), "infinite vector rejected");
    }

    void TetherProjection()
    {
        const Vec3 start{}, firstTip{0, 100, 0}, secondTip{40, 100, 0};
        TetherCurve resting;
        resting.Update(start, secondTip, 0.016, {0,1,0});
        // An abrupt pull moves both endpoints immediately while the interior settles over time.
        for (double rate : {30, 60, 144}) {
            TetherCurve moving;
            const double dt = 1 / rate;
            moving.Update(start, firstTip, dt, {0,1,0});
            moving.Update(start, secondTip, dt, {0,1,0});
            Check(Near(moving.Point(start, secondTip, 0), start) && Near(moving.Point(start, secondTip, 1), secondTip),
                "tether endpoints keep contact at once");
            Check(moving.Point(start, secondTip, 0.5).x > resting.Point(start, secondTip, 0.5).x &&
                resting.Point(start, secondTip, 0.5).z > 0, "both motion bend and resting bow face the opposite way");
            Check(Length(*moving.first.point - *resting.first.point) > 2 && Length(*moving.second.point - *resting.second.point) > 5, "tether interior lags on a quick pull");
            for (int i = 0; i < static_cast<int>(rate); ++i) moving.Update(start, secondTip, dt, {0,1,0});
            Check(Near(*moving.first.point, *resting.first.point, 0.01) && Near(*moving.second.point, *resting.second.point, 0.01), "tether interior settles at multiple frame rates");
        }

        const Vec3 p0{}, p1{0,4,0}, p2{8,4,0}, p3{8,0,0};
        Check(Near(CubicBezier(p0,p1,p2,p3,0.25), {1.25,2.25,0}), "cubic uses all four Bernstein weights");
        Check(Near(CubicBezier(p0,p1,p2,p3,-0.2), p0) && Near(CubicBezier(p0,p1,p2,p3,1.5), p3),
            "cubic evaluation clamps the fraction at both endpoints");
        const double epsilon = 1e-5;
        Check(Near((resting.Point(start,secondTip,epsilon) - start) / epsilon,
              (*resting.first.point - start) * 3, 0.01), "emitter tangent follows its own control point");
        Check(Near((secondTip - resting.Point(start,secondTip,1-epsilon)) / epsilon,
              (secondTip - *resting.second.point) * 3, 0.01), "tip tangent follows the second control point");
        Check(Length(secondTip - *resting.second.point * 3 + *resting.first.point * 3 - start) > 1,
            "resting beam is genuinely cubic");

        // Clip before perspective division when the tip leaves the view or crosses the near plane.
        const auto cut = ClipTetherSegment({0, 0, 10}, {15, 0, 10}, 2);
        Check(cut && Near(cut->start, {0, 0, 1}) && Near(cut->end, {1, 0, 1}), "off-screen tip clips at viewport");
        const auto behind = ClipTetherSegment({0, 0, 10}, {0.5, 0, -10}, 2);
        Check(behind && Near(behind->end, {0.1, 0, 1}), "tether clips before division at the camera near plane");
        Check(!ClipTetherSegment({20, 0, 10}, {15, 0, 10}, 2), "fully off-screen segment is hidden");
        Check(!ClipTetherSegment({0, 0, -1}, {0, 0, -10}, 2), "behind-camera segment is hidden");
    }

    void HandleSelection()
    {
        DragView::Camera camera{};
        const Vec3 eye{12,34,56};
        camera.x = {1,0,0,-eye.x};
        camera.y = {0,0,1,-eye.z};
        camera.w = {0,1,0,-eye.y};
        camera.width = 1920; camera.height = 1080; camera.nearPlane = 2;
        Check(camera.SetAim(eye) && Near(camera.aim.direction, {0,1,0}), "aim points into the displayed world");
        std::vector<std::optional<Vec3>> points(18);
        for (std::size_t i = 0; i < points.size(); ++i) {
            points[i] = eye + Vec3{0,100,0};
            DragView::Frame frame{.visible = true};
            camera.Project(points, frame);
            Check(frame.hover == i && frame.markers[i], "every mapped segment can independently be picked");
            points[i].reset();
        }
        points[2] = eye + Vec3{0,100,0};
        points[10] = eye + Vec3{0,50,0};
        points[6] = eye + Vec3{0,-5,0};
        DragView::Frame frame{.visible = true};
        camera.Project(points, frame);
        Check(frame.hover == 10 && !frame.markers[6], "nearer overlapping handle wins, behind-camera handle hidden");
        // Once held, the beam follows that handle even when another one is under the reticle.
        *points[10] = eye + Vec3{250,50,0};
        TetherCurve curve;
        curve.Update(camera.emitter, *points[10], 1.0/60, camera.aim.direction);
        DragView::Frame held{.visible = true};
        camera.Project(points, held);
        held.held = 10;
        camera.ProjectTether(curve, *points[10], held);
        Check(held.hover == 2 && held.held == 10 && !held.markers[10] && held.count > 0,
            "looking at a different handle preserves the clipped tether to the selected body");
        Check(held.orb && Near(*held.orb, {0.43,-0.5,1}) && Near(*held.orb, held.tether[0].start),
            "the visible orb is exactly the start of the clipped beam");
        DragView::Frame empty{.visible = true};
        camera.Project({}, empty);
        Check(empty.visible && !empty.hover && !empty.count, "editor reticle can draw with zero handles");
        camera.x[1] = 0.25; camera.x[3] -= 0.25 * eye.y;
        Check(camera.SetAim(eye), "offset viewport has a center ray");
        const auto center = camera.Clip(eye + camera.aim.direction * 100);
        Check(Near(center.x, 0) && Near(center.y, 0) && center.z > 0, "reticle ray matches shifted projection");
    }

    void CameraRelativeGrab()
    {
        const Vec3 right{1,0,0}, up{0,0,1};
        Ray aim{{10,0,30}, {0,1,0}};
        const Vec3 tip{13,80,28};
        auto hold = HoldDistance::Capture(aim, right, up, tip);
        Check(hold && Near(hold->Target(aim, right, up), tip), "pickup keeps the original point under an off-center handle");
        // The offset picked under the reticle stays in camera space as the view moves.
        aim.origin.y -= 1200;
        Check(Near(hold->Target(aim, right, up), tip - Vec3{0,1200,0}), "backing straight away translates target at retained depth");
        aim.direction = {1,0,0};
        Check(Near(hold->Target(aim, {0,-1,0}, up), aim.origin + Vec3{80,-3,-2}), "camera yaw preserves the pickup depth");

        // Screen-relative movement resolves to ground axes even when the camera pitches vertically.
        Check(Near(PoseTravel({0, 1, 1}, {1, 0, 0}, 1, 0, 1, 1.0 / 90), {0, 1, 1}),
            "arrow movement remains horizontal while Jump raises the pose");
        Check(Near(PoseTravel({0, 0, 1}, {1, 0, 0}, 1, 0, -1, 1.0 / 90), {0, 1, -1}),
            "looking directly up uses camera right to resolve the ground heading");
        Check(Near(Length(PoseTravel({0, 1, 0}, {1, 0, 0}, 1, 1, 0, 1.0 / 90)), 1),
            "diagonal arrow movement preserves horizontal speed");

        hold->Nudge(1);
        Check(Near(hold->Target(aim, {0,-1,0}, up), aim.origin + Vec3{82,-3,-2}),
            "one wheel notch moves only capture depth by two game units");
        hold->Nudge(-0.5);
        Check(Near(hold->depth, 81), "fractional wheel travel supports fine depth control");
        hold->depth = 12;
        hold->Nudge(-4);
        Check(Near(hold->depth, 10), "wheel motion keeps the grip in front of the camera");

        const auto initial = AxisAngle(right, 0.2);
        auto rotation = GripRotation::Begin(initial, right, up, {0,1,0});
        Check(rotation.has_value(), "screen-space rotation starts from the current pose");
        const auto yawed = rotation->Update(80, 0);
        const auto turned = AngularCorrection(initial, yawed, 1, 10);
        Check(turned && Near(*turned, up * -0.64), "right drag yaws around screen up");
        const auto reversedSign = Quaternion{-yawed.x,-yawed.y,-yawed.z,-yawed.w};
        Check(Near(*AngularCorrection(reversedSign, yawed, 16, 8), {}), "opposite-sign quaternion has no rotational error");
        auto pitch = GripRotation::Begin(initial, right, up, {0,1,0});
        const auto pitched = pitch->Update(0, 25);
        Check(Near(*AngularCorrection(initial, pitched, 1, 10), right * -0.2), "vertical torso drag tilts about screen right");
        const auto turnedLeft = rotation->Update(-160, 0);
        Check(Near(*AngularCorrection(initial, turnedLeft, 1, 10), up * 0.64), "left drag yaws around screen up in the opposite direction");

        const Vec3 forward{0,1,0};
        auto roll = GripRotation::Begin({}, right, up, forward);
        const auto rolled = roll->Update(0, 0, 1);
        Check(Near(*AngularCorrection({}, rolled, 1, 10), forward * (std::numbers::pi / 36)),
            "wheel up rolls clockwise around the view axis");
        Check(Near(*AngularCorrection({}, roll->Update(0, 0, -1), 1, 10), {}),
            "wheel down reverses roll");
        const auto fineTurn = roll->Update(40, 30, 0, 0.1);
        Check(Near(*AngularCorrection({}, fineTurn, 1, 10), up * -0.032 + right * -0.024),
            "10 percent sensitivity scales both mouse rotation axes");
        const auto fastRoll = roll->Update(0, 0, 1, 2);
        Check(Near(*AngularCorrection(fineTurn, fastRoll, 1, 10), forward * (std::numbers::pi / 18)),
            "200 percent sensitivity doubles wheel roll");
        auto combined = GripRotation::Begin(initial, right, up, forward);
        const auto afterYaw = combined->Update(40, 0);
        const auto afterRoll = combined->Update(0, 0, 2);
        Check(Near(*AngularCorrection(afterYaw, afterRoll, 1, 10), forward * (std::numbers::pi / 18)),
            "roll stays on the view axis after yaw");
        const auto forearm = *Unit(Vec3{2, 3, -4});
        auto forearmTwist = GripRotation::Begin(initial, right, up, forearm);
        const auto tiltedHelper = forearmTwist->Update(12, -5);
        const auto rolledHelper = forearmTwist->Update(0, 0, 1);
        Check(Near(*AngularCorrection(tiltedHelper, rolledHelper, 1, 10), forearm * (std::numbers::pi / 36)),
            "a twist helper scrolls along the arm independently of camera-relative tilt");
        const auto nudgedHelper = forearmTwist->Update(0, 0, 0, 0.5, {0, 0, 0.1}, forward);
        Check(Near(*AngularCorrection(rolledHelper, nudgedHelper, 1, 10), forward * 0.05),
            "keyboard roll uses the captured view axis and sensitivity on twist helpers");
        auto keyboard = GripRotation::Begin(initial, right, up, forward);
        const auto nudged = keyboard->Update(0, 0, 0, 1, {0.02, -0.03, 0});
        Check(Near(*AngularCorrection(initial, nudged, 1, 10), up * 0.02 - right * 0.03),
            "keyboard yaw and pitch share the held rotation axes");
        // Inversion applies independently to each axis for mouse and keyboard rotation.
        auto reverseYaw = GripRotation::Begin(initial, right, up, forward);
        const auto yawOnly = reverseYaw->Update(10, 10, 0, 1, {-0.02, -0.03, 0}, {}, {-1, 1, 1});
        Check(Near(*AngularCorrection(initial, yawOnly, 1, 10), up * 0.10 - right * 0.11),
            "yaw inversion reverses mouse and keyboard yaw independently");
        auto reversePitch = GripRotation::Begin(initial, right, up, forward);
        const auto pitchOnly = reversePitch->Update(10, 10, 0, 1, {-0.02, -0.03, 0}, {}, {1, -1, 1});
        Check(Near(*AngularCorrection(initial, pitchOnly, 1, 10), up * -0.10 + right * 0.11),
            "pitch inversion reverses mouse and keyboard pitch independently");
        auto reverseRoll = GripRotation::Begin(initial, right, up, forearm);
        const auto reversedWheel = reverseRoll->Update(0, 0, 1, 1, {}, {}, {1, 1, -1});
        Check(Near(*AngularCorrection(initial, reversedWheel, 1, 10), forearm * (-std::numbers::pi / 36)),
            "roll inversion reverses the wheel or its alternate binding");
        const auto reversedKey = reverseRoll->Update(0, 0, 0, 1, {0, 0, 0.1}, forward, {1, 1, -1});
        Check(Near(*AngularCorrection(reversedWheel, reversedKey, 1, 10), forward * -0.1),
            "roll inversion also reverses held keyboard nudges");
        const Vec3 contact{10,20,30}, localPoint{2,3,4};
        // A camera with a different orientation still uses its own axes.
        const Vec3 obliqueRight{0,1,0}, obliqueUp{0,0,1}, obliqueForward{-1,0,0};
        auto wrist = GripRotation::Begin(initial, obliqueRight, obliqueUp, obliqueForward);
        const auto pitchedWrist = wrist->Update(0, 70);
        Check(Near(*AngularCorrection(initial, pitchedWrist, 1, 10), obliqueRight * -0.56),
            "endpoint pitch follows the current camera right axis");
        const auto twisted = wrist->Update(0, 0, 1);
        Check(Near(*AngularCorrection(pitchedWrist, twisted, 1, 10), obliqueForward * (std::numbers::pi / 36)),
            "endpoint wheel roll follows the current camera forward axis");
        const auto pose = GripPose(contact, localPoint, twisted);
        Check(pose && Near(pose->position + *Rotate(pose->rotation, localPoint), contact), "wrist rotation keeps an off-center grip at the target");
        Check(!GripPose(contact, localPoint, {0,0,0,0}), "grip rejects invalid rotation");
    }

    void MouseInput()
    {
        GrabButton button;
        const auto tick = [&](bool focused, std::optional<bool> raw, MouseEdge edge = MouseEdge::None) {
            return button.Update(focused, raw, edge);
        };
        Check(!tick(true, false) && button.armed, "initial release arms selection");
        Check(tick(true, false, MouseEdge::Down) && button.held, "delivered down wins over stale sample");
        Check(!tick(true, true) && button.held, "matching later device down does not grab twice");
        Check(!tick(true, false) && !button.held, "sampled release works without delivered event");
        Check(tick(true, true) && button.held, "sampled press works in an unmapped context");
        // A button held through focus loss must be released before it can grip again.
        Check(!tick(false, true) && !button.held, "focus loss drops the grip");
        Check(!tick(true, true) && !button.held && !button.armed, "held across focus stays disarmed");
        Check(!tick(true, false) && button.armed, "release rearms after focus returns");
        Check(tick(true, true), "raw fallback starts a fresh grip");
        Check(!tick(true, true, MouseEdge::Down), "a delivered press already seen by polling cannot grab again");
        Check(!tick(true, std::nullopt, MouseEdge::Up) && !button.held, "native events work without raw device");

        EditorButtons buttons;
        EditorButtons::Samples raw{};
        const ButtonChange left{EditorButton::Grab, MouseEdge::Down}, drop{EditorButton::Grab, MouseEdge::Up};
        const ButtonChange right{EditorButton::Rotate, MouseEdge::Down}, up{EditorButton::Rotate, MouseEdge::Up};
        const ButtonChange guide{EditorButton::Guide, MouseEdge::Down}, settings{EditorButton::Settings, MouseEdge::Down};
        const auto frame = [&](std::initializer_list<ButtonChange> events, std::initializer_list<ButtonChange> expected,
                               const char* message, bool focused = true) {
            std::vector<ButtonChange> result;
            buttons.Update(focused, raw, events, [&](EditorButton key, MouseEdge edge) { result.push_back({key, edge}); });
            Check(std::ranges::equal(result, expected), message);
        };
        frame({}, {}, "initial release arms the editor buttons");
        // A frame may receive several ordered press and release events before it can poll.
        frame({right, up, right, up}, {right, up, right, up}, "both rapid toggles survive one tick");
        frame({left, right, up, drop}, {left, right, up, drop}, "left press precedes a later rotation grip");
        frame({right, left, drop, up}, {right, left, drop, up}, "a right rotation and a later left drag keep their order");
        frame({settings, right, up, guide}, {settings, right, up, guide}, "overlay commands preserve mouse event order");
        // Polling can see a press before its native event arrives. Deliver it once.
        raw[static_cast<std::size_t>(EditorButton::Rotate)] = true;
        frame({}, {right}, "a polled press toggles once");
        frame({right}, {}, "a delayed native down cannot toggle back");
        frame({}, {}, "holding cannot repeat a toggle");
        frame({right, up}, {up, {EditorButton::Guide, MouseEdge::Up}, {EditorButton::Settings, MouseEdge::Up}},
              "losing focus releases every held control and discards clicks", false);
        frame({right}, {}, "a held click cannot cross a focus change");
        raw[static_cast<std::size_t>(EditorButton::Rotate)] = false;
        frame({}, {}, "sampled release rearms the button");
        frame({right, up}, {right, up}, "first fresh click after sampled release works");
    }
}

int main()
{
    RotationAndSpace(); InverseKinematics(); NaturalLimbBends(); PlantedWristBend(); LimbCollisionClearance(); CoordinatedArmDrag(); AbsoluteWristSolve(); LearnedAnkleAndChestStance();
    ClavicleRecovery(); EndpointRotationIK(); WristTwist();
    HierarchyReconstruction(); TwistHierarchyReconstruction();
    ReplicatedPoseTranslation(); CameraLook(); CameraGazeAndMirror(); PoseHistoryActions(); SharedPoseHistory(); TestPresetFiles();
    FingerGestures(); FingerDiagramDrag(); FingerGestureGeometry();
    WholePoseRotation(); PlantedEndpoints(); IdleBreathing(); DraggingAndPicking(); TetherProjection(); HandleSelection(); CameraRelativeGrab(); MouseInput(); InvalidNumbers();
    std::cout << "PASS: " << assertions << " pose math assertions (UBSan enabled)\n";
}
