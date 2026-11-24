#pragma once

#include "EndpointIK.h"

#include <filesystem>
#include <fstream>
#include <limits>

namespace GModPose
{
    class EndpointModel
    {
    public:
        static constexpr std::size_t modes = 3;
        struct Predictions
        {
            std::array<EndpointCandidate, modes> candidates;
            std::size_t count{};

            std::span<const EndpointCandidate> View() const { return std::span(candidates).first(count); }
        };

        bool Load(const std::filesystem::path& path, bool leg = false)
        {
            parameters.clear();
            std::ifstream file(path, std::ios::binary);
            std::array<char, 8> signature{};
            std::array<std::uint32_t, 2> dimensions{};
            if (!file.read(signature.data(), signature.size()) || signature != (leg ? legFormat : armFormat) ||
                !file.read(reinterpret_cast<char*>(dimensions.data()), sizeof(dimensions))) return false;
            const auto [width, hiddenLayers] = dimensions;
            if (width < 32 || width > maxWidth || hiddenLayers < 1 || hiddenLayers > 6) return false;
            const auto count = (inputCount + 1) * width + (hiddenLayers - 1) * (width + 1) * width +
                (width + 1) * outputCount;
            std::vector<float> values(count);
            if (!file.read(reinterpret_cast<char*>(values.data()), static_cast<std::streamsize>(values.size() * sizeof(float))) ||
                file.peek() != std::char_traits<char>::eof() ||
                !std::ranges::all_of(values, [](float value) { return std::isfinite(value); })) return false;
            parameters = std::move(values);
            hiddenWidth = width;
            layers = hiddenLayers;
            return true;
        }

        Predictions Predict(const EndpointReference& rig, Vec3 target) const
        {
            Predictions result{};
            const double reach = rig.Reach();
            if (parameters.empty() || !std::isfinite(reach) || reach < .01) return result;
            const auto q = rig.frame.rotation;
            const Quaternion inverse{-q.x, -q.y, -q.z, q.w};
            // The weights expect rig-relative points in units of the limb's total reach.
            const std::array points{target, rig.context[0], rig.context[1]};
            std::array<float, inputCount> input{};
            for (std::size_t i = 0; i < points.size(); ++i) {
                const auto local = Rotate(inverse, (points[i] - rig.frame.position) / reach);
                if (!local || Length(*local) > 3) return result;
                input[3 * i] = static_cast<float>(local->x);
                input[3 * i + 1] = static_cast<float>(local->y);
                input[3 * i + 2] = static_cast<float>(local->z);
            }
            input.back() = rig.left ? -1.F : 1.F;

            std::span<const float> weights(parameters);
            std::array<float, maxWidth> a{}, b{};
            auto current = std::span(a).first(hiddenWidth);
            auto next = std::span(b).first(hiddenWidth);
            Layer(weights, input, current, true);
            for (std::size_t i = 1; i < layers; ++i) {
                Layer(weights, current, next, true);
                std::swap(current, next);
            }
            std::array<float, outputCount> output{};
            Layer(weights, current, output, false);

            for (std::size_t i = 0; i < modes; ++i) {
                const auto row = std::span(output).subspan(i * valuesPerMode, valuesPerMode);
                const auto vector = [&](std::size_t index) {
                    return Vec3{row[index], row[index + 1], row[index + 2]};
                };
                if (Length(vector(0)) > 3 || Length(vector(3)) > 3 || !std::isfinite(row.back())) continue;
                const auto shoulder = Rotate(q, vector(0) * reach), elbow = Rotate(q, vector(3) * reach);
                if (!shoulder || !elbow) continue;

                EndpointCandidate candidate{rig.frame.position + *shoulder, rig.frame.position + *elbow, {}, row.back()};
                bool valid = true;
                for (std::size_t joint = 0; joint < candidate.rotations.size(); ++joint) {
                    const auto index = 6 + 6 * joint;
                    const auto delta = DecodeAxes(vector(index), vector(index + 3));
                    if (!delta) {
                        valid = false;
                        break;
                    }
                    candidate.rotations[joint] = Multiply(Multiply(q, *delta), rig.frameBindRotations[joint]);
                }
                if (valid) result.candidates[result.count++] = candidate;
            }
            return result;
        }

    private:
        static constexpr std::array<char, 8> armFormat{'G', 'M', 'P', 'W', 'I', 'K', '0', '1'};
        static constexpr std::array<char, 8> legFormat{'G', 'M', 'P', 'A', 'I', 'K', '0', '1'};
        static constexpr std::size_t inputCount = 10, valuesPerMode = 25, outputCount = modes * valuesPerMode;
        static constexpr std::size_t maxWidth = 1024;
        static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
        std::size_t hiddenWidth{}, layers{};
        std::vector<float> parameters;

        static std::optional<Quaternion> DecodeAxes(Vec3 first, Vec3 second)
        {
            const auto x = Unit(first);
            if (!x) return std::nullopt;
            // The model predicts two basis vectors. Orthogonalize before recovering rotation.
            const auto y = BendOnPlane(second, *x);
            if (!y) return std::nullopt;
            const auto alignX = FromTo({1, 0, 0}, *x);
            const auto rotatedY = alignX ? Rotate(*alignX, {0, 1, 0}) : std::nullopt;
            if (!rotatedY) return std::nullopt;
            const auto roll = std::atan2(Dot(*x, Cross(*rotatedY, *y)), Dot(*rotatedY, *y));
            return Unit(Multiply(AxisAngle(*x, roll), *alignX));
        }

        static void Layer(std::span<const float>& parameters, std::span<const float> input,
                          std::span<float> output, bool relu)
        {
            const auto matrix = parameters.first(input.size() * output.size());
            const auto bias = parameters.subspan(matrix.size(), output.size());
            for (std::size_t i = 0; i < output.size(); ++i) {
                float value = bias[i];
                const auto row = matrix.subspan(i * input.size(), input.size());
                for (std::size_t j = 0; j < input.size(); ++j) value += row[j] * input[j];
                output[i] = relu ? std::max(value, 0.0F) : value;
            }
            parameters = parameters.subspan(matrix.size() + bias.size());
        }
    };
}
