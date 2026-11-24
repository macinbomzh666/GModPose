#include <RE/Skyrim.h>
#include "FacePanel.h"

#include <atomic>
#include <cmath>
#include <utility>

namespace GModPose::FaceActor
{
    namespace
    {
        using namespace FacePose;
        using Native = RE::BSFaceGenKeyframeMultiple;
        static_assert(Aah == modifierCount && Native::BlinkLeft == 0 && Native::SquintRight + 1 == modifierCount);
        static_assert(Native::Aah == 0 && Native::W + 1 == phonemeCount && Native::MoodNeutral == neutralMood);
        static_assert(std::tuple_size_v<Expressions> == Native::CombatShout + 1);

        struct Session
        {
            const RE::NiAVObject* root{};
            const RE::BSFaceGenAnimationData* identity{};
            Sample held;
            RE::BSFaceGenAnimationData::EyesBlinkingStage blinkingStage{};
            float blinkingTimer{};
        };
        // The editor update owns the session. Other plugins only read the
        // published flag through the editor-state export.
        std::optional<Session> session;
        std::atomic_bool ownsFace{false};

        bool Has(const Native& keyframe, std::size_t index)
        {
            return keyframe.values && index < keyframe.count;
        }

        float ReadWeight(const Native& input, const Native& output, std::size_t index)
        {
            // The rendered keyframe includes engine blends already visible on the face.
            for (const auto* keyframe : {&output, &input})
                if (Has(*keyframe, index) && std::isfinite(keyframe->values[index]))
                    return std::clamp(keyframe->values[index], 0.0F, 1.0F);
            return 0.0F;
        }

        void Write(Native& keyframe, std::size_t index, float value, bool refresh = false)
        {
            if (Has(keyframe, index) && (refresh || keyframe.values[index] != value))
                keyframe.SetValue(static_cast<std::uint32_t>(index), value);
        }

        template <class Data, class Visitor>
        void VisitWeights(Data& data, Visitor visit)
        {
            for (std::size_t i = 0; i < weightCount; ++i) {
                auto& input = i < modifierCount ? data.modifierKeyFrame : data.phenomeKeyFrame;
                auto& output = i < modifierCount ? data.modifier3 : data.phoneme3;
                const auto index = i < modifierCount ? i : i - modifierCount;
                visit(input, output, index, i, Mask{1} << i);
            }
        }

        void Capture(Session& current, const RE::BSFaceGenAnimationData& data)
        {
            auto& sample = current.held;
            VisitWeights(data, [&](const Native& input, const Native& output, auto index, auto i, Mask bit) {
                if (!Has(input, index) && !Has(output, index)) return;
                sample.pose.weights[i] = ReadWeight(input, output, index);
                sample.supported |= bit;
            });
            for (std::size_t i = 0; i < sample.expressions.size(); ++i)
                sample.expressions[i] = ReadWeight(data.expressionKeyFrame, data.expression3, i);
            constexpr auto lastMood = neutralMood + moodCount - 1;
            if (Has(data.expressionKeyFrame, lastMood) || Has(data.expression3, lastMood)) {
                sample.supported |= Bit(MoodSlot);
                for (auto i = neutralMood; i <= lastMood; ++i) {
                    if (const auto strength = sample.expressions[i]; strength > sample.pose.strength) {
                        sample.pose.mood = i;
                        sample.pose.strength = strength;
                    }
                }
            }
        }

        void WriteWeights(RE::BSFaceGenAnimationData& data, const Sample& held, Mask refresh)
        {
            VisitWeights(data, [&](Native& input, Native& output, auto index, auto i, Mask bit) {
                if (!(held.supported & bit)) return;
                Write(input, index, held.pose.weights[i], (refresh & bit) != 0);
                Write(output, index, held.pose.weights[i], (refresh & bit) != 0);
            });
        }

        void WriteExpressions(RE::BSFaceGenAnimationData& data, const Expressions& expressions, bool refresh)
        {
            for (std::size_t i = 0; i < expressions.size(); ++i) {
                Write(data.expressionKeyFrame, i, expressions[i], refresh);
                Write(data.expression3, i, expressions[i], refresh);
            }
        }
    }

    std::optional<Sample> Snapshot()
    {
        if (!session || !session->held.supported || !OwnsFace()) return std::nullopt;
        return session->held;
    }

    bool Update(RE::PlayerCharacter& player, const std::optional<FacePose::Edit>& edit)
    {
        auto* data = player.GetFaceGenAnimationData();
        const auto* root = player.Get3D(false);
        if (!data || !root || (session && session->root != root)) {
            ownsFace.store(false);
            return false;
        }

        RE::BSSpinLockGuard guard(data->lock);
        if (!session) {
            session.emplace();
            session->root = root;
            Capture(*session, *data);
            if (!session->held.supported && !Has(data->expressionKeyFrame, 0) && !Has(data->expression3, 0)) {
                session.reset();
                return false;
            }
        }

        auto& current = *session;
        const bool newFace = current.identity != data;
        if (newFace) {
            current.identity = data;
            current.blinkingStage = data->eyesBlinkingStage;
            current.blinkingTimer = data->eyesBlinkingTimer;
        }

        const auto touched = edit ? edit->changed & current.held.supported : 0;
        if (touched) Merge(current.held.pose, edit->pose, touched);
        const bool moodEdited = (touched & Bit(MoodSlot)) != 0;
        if (moodEdited) current.held.expressions = edit->expressions;

        if (current.held.supported & blink) {
            data->eyesBlinkingStage = RE::BSFaceGenAnimationData::EyesBlinkingStage::BlinkDelay;
            data->eyesBlinkingTimer = 0.4F;
        }
        const auto& face = current.held.pose;
        if ((newFace || moodEdited || !data->exprOverride) && Has(data->expressionKeyFrame, face.mood)) {
            data->ClearExpressionOverride();
            data->SetExpressionOverride(face.mood, face.strength);
            data->exprOverride = true;
        }
        WriteExpressions(*data, current.held.expressions, newFace || moodEdited);
        WriteWeights(*data, current.held, newFace ? all : touched);
        ownsFace.store(true);
        return true;
    }

    bool OwnsFace() noexcept
    {
        return ownsFace.load();
    }

    void End(RE::PlayerCharacter* player)
    {
        auto previous = std::exchange(session, std::nullopt);
        ownsFace.store(false);
        // Only release FaceGen values on the same actor model and face data.
        if (!previous || !player || player->Get3D(false) != previous->root) return;
        auto* data = player->GetFaceGenAnimationData();
        if (!data || data != previous->identity) return;
        RE::BSSpinLockGuard guard(data->lock);
        WriteWeights(*data, Sample{{}, previous->held.supported}, all);
        data->ClearExpressionOverride();
        WriteExpressions(*data, {}, true);
        if (previous->held.supported & blink) {
            data->eyesBlinkingStage = previous->blinkingStage;
            data->eyesBlinkingTimer = previous->blinkingTimer;
        }
    }
}
