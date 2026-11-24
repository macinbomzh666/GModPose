#pragma once

#include "LimbPose.h"

namespace GModPose
{
    struct EndpointReference
    {
        Transform frame;
        std::array<Vec3, 2> context;
        std::array<Transform, 3> joints;
        std::array<Quaternion, 3> frameBindRotations;
        Transform shoulderParent;
        std::optional<Transform> clavicle;
        std::optional<TorsoClearance> clearance;
        bool left{}, leg{};

        double Reach() const
        {
            return Length(joints[1].position - joints[0].position) + Length(joints[2].position - joints[1].position);
        }
    };

    struct EndpointCandidate
    {
        Vec3 shoulder, elbow;
        std::array<Quaternion, 3> rotations;
        double logit;
    };

    struct EndpointSolution
    {
        std::array<Transform, 3> joints;
        Quaternion clavicle, shoulderParent;
        bool limited;
    };

    inline Quaternion BoundClavicleSwing(Quaternion rotation, double limit)
    {
        const auto sign = rotation.w < 0 ? -1.0 : 1.0;
        const Vec3 imaginary{rotation.x * sign, rotation.y * sign, rotation.z * sign};
        const auto axis = Unit(imaginary);
        if (!axis) return {};
        const auto angle = 2 * std::atan2(Length(imaginary), std::abs(rotation.w));
        return AxisAngle(*axis, std::min(angle, limit));
    }

    inline std::optional<EndpointSolution> SolveEndpoint(const EndpointReference& rig, Vec3 target,
                                                   std::span<const EndpointCandidate> candidates = {},
                                                   std::optional<Quaternion> fixedRotation = {},
                                                   std::optional<Quaternion> relativeRotation = {})
    {
        // Use the captured rig reference for every target. The last solved bend is irrelevant.
        const auto& [shoulder, elbow, wrist] = rig.joints;
        const auto clavicle = rig.clavicle.value_or(rig.shoulderParent);
        const double reach = rig.Reach();
        if (!std::isfinite(reach) || reach < .01 || !IsFinite(target)) return std::nullopt;
        if (!Unit(rig.frame.rotation)) return std::nullopt;
        const auto right = *Rotate(rig.frame.rotation, {1, 0, 0}) * (rig.left ? -1 : 1);
        const auto up = *Rotate(rig.frame.rotation, {0, 0, 1});
        const auto forward = *Rotate(rig.frame.rotation, {0, 1, 0});
        const auto fallbackPole = rig.leg ? forward + right * .12 : right * .70 - up * .70 +
            forward * std::clamp(Dot(target - rig.frame.position, forward) / reach, -.70, .20);
        const auto bindHinge = CaptureLimbHinge(rig.joints, rig.leg ? fallbackPole : right * .25 - forward, rig.leg);
        const auto localWrist = RelativeTo(elbow, wrist);
        if (!bindHinge || !localWrist) return std::nullopt;
        std::array<Vec3, 2> localShafts;
        for (std::size_t i = 0; i < localShafts.size(); ++i) {
            const auto segment = RelativeTo(rig.joints[i], rig.joints[i + 1]);
            if (!segment) return std::nullopt;
            localShafts[i] = segment->position;
        }
        const auto shoulderShaft = shoulder.position - clavicle.position;
        const auto top = std::ranges::max_element(candidates, {}, &EndpointCandidate::logit);
        const double topLogit = top == candidates.end() ? 0.0 : top->logit;
        struct Choice
        {
            Vec3 root, elbow, wrist, bend;
            Quaternion clavicleSwing;
            const EndpointCandidate* prior;
            double wristError, cost, overlap;
        };

        const auto search = [&](const EndpointCandidate* prior) -> std::optional<Choice> {
            std::optional<Choice> best;
            const auto direction = [&](std::size_t segment) -> std::optional<Vec3> {
                const auto shaft = prior ? Rotate(prior->rotations[segment], localShafts[segment]) : std::nullopt;
                return shaft ? Unit(*shaft) : std::nullopt;
            };
            const auto upperDirection = direction(0), lowerDirection = direction(1);
            std::array<Quaternion, 11> rotations{};
            std::size_t count = 1;
            if (rig.clavicle) {
                constexpr double maxClavicleTurn = .42;
                const auto addSwing = [&](Vec3 goal, double limit) {
                    if (const auto swing = FromTo(shoulderShaft, goal))
                        rotations[count++] = BoundClavicleSwing(*swing, limit);
                };
                if (prior) {
                    addSwing(prior->shoulder - clavicle.position, maxClavicleTurn);
                    addSwing(prior->shoulder - clavicle.position, maxClavicleTurn * .5);
                }
                addSwing(target - clavicle.position, maxClavicleTurn);
                addSwing(clavicle.position - target, maxClavicleTurn);
                for (const auto axis : {right, up, forward})
                    for (const double angle : {-.24, .24}) rotations[count++] = AxisAngle(axis, angle);
            }

            // Project the proposed shoulder and elbow onto the chain's exact reach.
            for (const auto rotation : std::span(rotations).first(count)) {
                const auto origin = clavicle.position + *Rotate(rotation, shoulderShaft);
                const auto startElbow = origin + *Rotate(rotation, elbow.position - shoulder.position);
                const auto startWrist = startElbow + *Rotate(rotation, wrist.position - elbow.position);
                const auto pole = prior ? prior->elbow : origin + fallbackPole * reach;
                const auto solved = SolveTwoBone({origin, startElbow, startWrist, target, pole, {}});
                if (!solved) continue;
                const double error = std::max(Length(solved->tip - target) - 1e-6, 0.0);
                if (best && error > best->wristError) continue;
                const auto axis = Unit(solved->tip - origin);
                if (!axis) continue;
                std::optional<Vec3> kneeDirection;
                if (rig.leg) {
                    const auto swing = FromTo(startWrist - origin, *axis);
                    const auto facing = swing ? Rotate(*swing, fallbackPole) : std::nullopt;
                    if (facing) kneeDirection = BendOnPlane(*facing, *axis);
                }
                // Swivel around this circle to improve clearance while the endpoint stays fixed.
                const auto circleCenter = origin + *axis * Dot(solved->joint - origin, *axis);
                const auto radius = Length(solved->joint - circleCenter);
                const auto tangent = Cross(*axis, solved->bend);

                const double clavicleCost = .16 * (1 - std::abs(rotation.w));
                const double priorCost = prior ? .045 * std::min(topLogit - prior->logit, 12.0) +
                    6 * Dot(origin - prior->shoulder, origin - prior->shoulder) / (reach * reach)
                    : .10 * Dot(origin - shoulder.position, origin - shoulder.position) / (reach * reach);
                const double upperLength = Length(startElbow - origin), lowerLength = Length(startWrist - startElbow);
                const auto score = [&](double swivel) {
                    const auto bend = solved->bend * std::cos(swivel) + tangent * std::sin(swivel);
                    const auto joint = circleCenter + bend * radius;
                    // Knee flexion stays on the forward side of the leg.
                    const double reverse = kneeDirection && Dot(bend, *kneeDirection) < 0 ? 100.0 : 0.0;
                    const double clearance = rig.clearance ? rig.clearance->LimbOverlap(origin, joint, solved->tip) : 0.0;
                    const double deviation = prior ?
                        5 * Dot(joint - prior->elbow, joint - prior->elbow) / (reach * reach) : 0;
                    // Penalize candidates that require a large swing away from
                    // the learned orientations to land both limb segments.
                    const double alignment = upperDirection && lowerDirection ?
                        .18 * (2 - Dot(*upperDirection, joint - origin) / upperLength -
                            Dot(*lowerDirection, solved->tip - joint) / lowerLength) : 0;
                    return Choice{origin, joint, solved->tip, bend, rotation, prior, error,
                        reverse + 4 * clearance + deviation + alignment + clavicleCost + priorCost + .14 * (1 - std::cos(swivel)),
                        clearance};
                };
                constexpr double step = std::numbers::pi / 12;
                double swivel = 0;
                auto local = score(swivel);
                if (radius > .001) {
                    for (int i = -11; i <= 12; ++i) {
                        const auto trial = score(i * step);
                        if (trial.cost < local.cost) {
                            local = trial;
                            swivel = i * step;
                        }
                    }
                    for (const auto refinement : {step * .5, step * .25, step * .125}) {
                        const auto left = score(swivel - refinement), right = score(swivel + refinement);
                        if (left.cost < local.cost && left.cost <= right.cost) {
                            local = left;
                            swivel -= refinement;
                        } else if (right.cost < local.cost) {
                            local = right;
                            swivel += refinement;
                        }
                    }
                }
                if (!best || error < best->wristError || local.cost < best->cost)
                    best = local;
            }
            return best;
        };

        std::vector<Choice> choices;
        choices.reserve(candidates.size());
        for (const auto& prior : candidates)
            if (const auto choice = search(&prior)) choices.push_back(*choice);
        const auto closest = std::ranges::min_element(choices, [](const Choice& a, const Choice& b) {
            if (a.wristError != b.wristError) return a.wristError < b.wristError;
            return a.cost < b.cost;
        });
        auto best = closest == choices.end() ? search(nullptr) : std::optional{*closest};
        if (!best) return std::nullopt;

        EndpointCandidate blended{{}, {}, {}, topLogit};
        if (choices.size() > 1 && best->wristError == 0) {
            // Close scores often represent different valid elbow bends. A
            // hard winner can jump as the pointer crosses their boundary.
            // Blend the priors, then project again at the identical target.
            std::vector<double> weights;
            weights.reserve(choices.size());
            double total = 0;
            for (const auto& choice : choices) {
                const double weight = choice.wristError == 0 ? std::exp((best->cost - choice.cost) / .18) : 0;
                weights.push_back(weight);
                total += weight;
                blended.shoulder = blended.shoulder + choice.prior->shoulder * weight;
                blended.elbow = blended.elbow + choice.prior->elbow * weight;
            }
            blended.shoulder = blended.shoulder / total;
            blended.elbow = blended.elbow / total;
            for (std::size_t i = 0; i < blended.rotations.size(); ++i) {
                auto mean = best->prior->rotations[i];
                // Power iteration on the quaternion outer products avoids
                // treating opposite signs as different rotations.
                for (int iteration = 0; iteration < 12; ++iteration) {
                    Quaternion sum{0, 0, 0, 0};
                    for (std::size_t j = 0; j < choices.size(); ++j) {
                        const auto q = choices[j].prior->rotations[i];
                        const double factor = weights[j] * (q.x * mean.x + q.y * mean.y + q.z * mean.z + q.w * mean.w);
                        sum = {sum.x + q.x * factor, sum.y + q.y * factor, sum.z + q.z * factor, sum.w + q.w * factor};
                    }
                    mean = Unit(sum).value_or(mean);
                }
                blended.rotations[i] = mean;
            }
            if (const auto projected = search(&blended);
                projected && projected->wristError == 0 && projected->overlap <= best->overlap + .015)
                best = projected;
        }

        const auto rotation = best->clavicleSwing;
        const auto normal = Unit(Cross(best->bend, best->wrist - best->root));
        const std::array solvedSegments{best->elbow - best->root, best->wrist - best->elbow};
        EndpointSolution result{{Transform{best->root, {}}, Transform{best->elbow, {}}, Transform{best->wrist, {}}},
            Multiply(rotation, clavicle.rotation), Multiply(rotation, rig.shoulderParent.rotation),
            best->wristError > .025};
        for (std::size_t i = 0; i < solvedSegments.size(); ++i) {
            const auto from = best->prior ? best->prior->rotations[i] : Multiply(rotation, rig.joints[i].rotation);
            const auto shaft = Rotate(from, localShafts[i]);
            const auto swing = shaft ? FromTo(*shaft, solvedSegments[i]) : std::nullopt;
            if (!swing) return std::nullopt;
            auto aligned = *swing;
            if (!best->prior && normal)
                aligned = FollowLimbPlane(aligned, from, bindHinge->localNormals[i], solvedSegments[i], *normal);
            result.joints[i].rotation = Multiply(aligned, from);
            if (i == 1) {
                // Torso moves can pin a world rotation. A direct drag can keep only joint flex.
                result.joints[2].rotation = fixedRotation.value_or(
                    relativeRotation ? Multiply(result.joints[1].rotation, *relativeRotation) :
                    best->prior ? Multiply(aligned, best->prior->rotations[2]) :
                    Multiply(result.joints[1].rotation, localWrist->rotation));
            }
        }
        return result;
    }

    inline std::optional<SegmentTwistFrame> EndpointTwistFrame(const EndpointReference& rig, const EndpointSolution& solved,
                                                            std::size_t segment)
    {
        if (segment > 1) return std::nullopt;
        const auto parent = segment == 0 ? rig.shoulderParent.rotation : rig.joints[0].rotation;
        const auto newParent = segment == 0 ? solved.shoulderParent : solved.joints[0].rotation;
        const auto distal = RelativeTo(rig.joints[segment], rig.joints[segment + 1]);
        const auto newDistal = RelativeTo(solved.joints[segment], solved.joints[segment + 1]);
        if (!distal || !newDistal) return std::nullopt;
        SegmentTwistFrame frame;
        if (!frame.Begin(Multiply({-parent.x, -parent.y, -parent.z, parent.w}, rig.joints[segment].rotation), *distal))
            return std::nullopt;
        frame.Measure(Multiply({-newParent.x, -newParent.y, -newParent.z, newParent.w},
            solved.joints[segment].rotation), newDistal->rotation);
        return frame;
    }
}
