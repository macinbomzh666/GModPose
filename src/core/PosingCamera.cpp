#include <RE/Skyrim.h>
#include <SKSE/SKSE.h>
#include "PosingCamera.h"
#include "Editor.h"
#include "DragView.h"
#include "HookOriginal.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>

namespace GModPose::PosingCamera
{
    namespace
    {
        using Clock = std::chrono::steady_clock;
        using Mode = RE::CameraState;
        constexpr auto kTFC = RE::ControlMap::InputContextID::kTFCMode;

        struct Viewpoint
        {
            RE::NiPoint3 position;
            Quaternion rotation;
            Vec3 forward, right;
            RE::BSTPoint2<float> angles;
        };

        struct Session
        {
            RE::PlayerCamera* camera{};
            const RE::PlayerCharacter* player{};
            const RE::TESObjectCELL* cell{};
            const RE::NiAVObject* root{};
            RE::ObjectRefHandle seat;
            unsigned world{}, attempts{};
            Mode entry{}, last{};
            std::optional<Viewpoint> view;
            Clock::time_point attempted{};
            RE::NiPoint3 entryFreePosition{};
            RE::BSTPoint2<float> entryFreeAngles{};
            enum class AngleAxis { Pitch, Yaw };
            struct AngleBinding { AngleAxis axis; double sign; };
            std::array<AngleBinding, 2> angles{};
            bool nativeAnglesValid{}, flightReported{}, translationReported{};
            double radiansPerCount{0.0015}, verticalSign{1};
            bool acquired{}, nativeEntry{}, entryContext{};
        } session;
        std::atomic_uint world{0};
        std::atomic<RE::FreeCameraState*> protectedState{nullptr};
        std::atomic<RE::TESCameraState*> renderViewState{nullptr};

        bool FollowCamera(Mode id, bool seated)
        {
            return id == Mode::kThirdPerson || id == Mode::kFirstPerson || id == Mode::kBleedout ||
                (seated && id == Mode::kFurniture);
        }

        template <class Handler>
        struct GameplayInput
        {
            static bool Thunk(Handler* self, RE::InputEvent* event)
            {
                return (!Runtime::Running() || DragView::InputBlockReason()) && original(self, event);
            }

            static void Install()
            {
                REL::Relocation<std::uintptr_t> table{Handler::VTABLE[0]};
                if (!original.InstallVirtual(table, 1, Thunk))  // PlayerInputHandler::CanProcess
                    SKSE::log::warn("Could not install camera gameplay input hook at 0x{:X}", table.address());
            }

            static inline HookOriginal<decltype(Thunk)> original;
        };

        struct FreeInput
        {
            static bool Thunk(RE::PlayerInputHandler* self, RE::InputEvent* event)
            {
                const auto* free = protectedState.load();
                return !(free && static_cast<const RE::PlayerInputHandler*>(free) == self && Runtime::Running()) &&
                    original(self, event);
            }
            static inline HookOriginal<decltype(Thunk)> original;
        };

        struct FreeRotation
        {
            static void Thunk(RE::FreeCameraState* self, RE::NiQuaternion& result)
            {
                if (protectedState.load() == self && session.view) {
                    const auto q = session.view->rotation;
                    result = {float(q.w), float(q.x), float(q.y), float(q.z)};
                    return;
                }
                original(self, result);
            }
            static inline HookOriginal<decltype(Thunk)> original;
        };

        struct RenderCamera
        {
            // Override the rendered view for seated flight. Keeping the native
            // furniture camera state avoids a furniture transition.
            static void Thunk(RE::NiCamera* camera, RE::NiUpdateData* data)
            {
                if (target.load() != camera || !Runtime::Running()) return original(camera, data);
                std::lock_guard lock(mutex);
                if (target.load() != camera) return original(camera, data);
                auto world = camera->world;
                world.translate = position;
                world.rotate = rotation;
                const auto nativeLocal = camera->local;
                const auto* parent = camera->parent;
                if (parent && (!std::isfinite(parent->world.scale) || std::abs(parent->world.scale) < 1e-6F))
                    return original(camera, data);
                // Feed the desired world transform through the engine's local-to-world update.
                camera->local = parent ? parent->world.Invert() * world : world;
                original(camera, data);
                camera->local = nativeLocal;
            }

            static void Publish(const Viewpoint& view)
            {
                auto* camera = RE::Main::WorldRootCamera();
                if (!camera) return;
                {
                    std::lock_guard lock(mutex);
                    position = view.position;
                    const auto q = view.rotation;
                    RE::NiQuaternion{float(q.w), float(q.x), float(q.y), float(q.z)}.ToRotation(rotation);
                    target.store(camera);
                }
                // Publish before the editor samples the projection this frame.
                RE::NiUpdateData update{};
                Thunk(camera, &update);
            }

            static void Clear(bool restore)
            {
                if (!target.load()) return;
                std::lock_guard lock(mutex);
                auto* camera = target.exchange(nullptr);
                if (!restore || !camera || camera != RE::Main::WorldRootCamera()) return;
                RE::NiUpdateData update{};
                original(camera, &update);
            }

            static inline std::atomic<RE::NiCamera*> target{nullptr};
            static inline std::mutex mutex;
            static inline RE::NiPoint3 position;
            static inline RE::NiMatrix3 rotation;
            static inline HookOriginal<decltype(Thunk)> original;
        };

        struct RenderCameraInput
        {
            static bool Thunk(RE::PlayerInputHandler* self, RE::InputEvent* event)
            {
                const auto* state = renderViewState.load();
                if (state && state->id == Mode::kThirdPerson && Runtime::Running() &&
                    static_cast<const RE::PlayerInputHandler*>(static_cast<const RE::ThirdPersonState*>(state)) == self &&
                    !DragView::InputBlockReason()) return false;
                return original(self, event);
            }

            static inline HookOriginal<decltype(Thunk)> original;
        };

        struct FreeUpdate
        {
            static void Thunk(RE::FreeCameraState* self, RE::BSTSmartPointer<RE::TESCameraState>& next)
            {
                if (protectedState.load() != self || !Runtime::Running() || !session.view) return original(self, next);
                const auto& view = *session.view;
                self->translation = view.position;
                self->rotation = view.angles;
                self->verticalDirection = 0;
                self->zUpDown = {};
                if (auto* camera = RE::PlayerCamera::GetSingleton()) camera->rotationInput = {};
                if (auto* controls = RE::PlayerControls::GetSingleton()) {
                    controls->data.moveInputVec = {};
                    controls->data.lookInputVec = {};
                    controls->data.prevLookVec = {};
                }
                // Keep the native free-state update running so the engine can
                // update the view after consuming the editor's transform.
                original(self, next);
                self->translation = view.position;
                self->rotation = view.angles;
                if (next && FollowCamera(next->id, static_cast<bool>(session.seat))) next.reset(self);
            }
            static inline HookOriginal<decltype(Thunk)> original;
        };

        bool HasFreeInput(const RE::ControlMap& map)
        {
            const auto& stack = map.GetRuntimeData().contextPriorityStack;
            return std::find(stack.begin(), stack.end(), kTFC) != stack.end();
        }

        void RemoveFreeInput()
        {
            if (auto* map = RE::ControlMap::GetSingleton(); map && HasFreeInput(*map))
                map->PopInputContext(kTFC);
        }

        std::optional<Quaternion> NativeRotation(RE::FreeCameraState& free)
        {
            RE::NiQuaternion native;
            FreeRotation::original(&free, native);
            return Unit(Quaternion{native.x, native.y, native.z, native.w});
        }

        bool SeedView(RE::FreeCameraState& free, const DragView::Camera& rendered)
        {
            const auto rotation = NativeRotation(free);
            const auto& position = free.translation;
            if (!rotation || !IsFinite(Vec3{position.x, position.y, position.z})) return false;
            const auto horizontal = Unit(Cross(rendered.aim.direction, Vec3{0, 0, 1}));
            const auto right = horizontal.value_or(rendered.right);
            const auto angles = free.rotation;
            session.view = Viewpoint{position, *rotation, rendered.aim.direction, right, angles};

            // Identify the native angle axes from the free camera's own getter.
            // Some engine paths can consume the native angles directly.
            const auto [x, y, z, w] = *rotation;
            const Quaternion inverse{-x, -y, -z, w};
            using Axis = Session::AngleAxis;
            const auto probe = [&](bool first) -> std::optional<Session::AngleBinding> {
                constexpr float step = 0.001F;
                if (first) free.rotation.x += step;
                else free.rotation.y += step;
                const auto moved = NativeRotation(free);
                free.rotation = angles;
                if (!moved) return std::nullopt;
                auto delta = Multiply(*moved, inverse);
                const auto axis = Unit(Vec3{delta.x, delta.y, delta.z} * (delta.w < 0 ? -1 : 1));
                if (!axis) return std::nullopt;
                const auto across = Dot(*axis, right);
                if (std::abs(axis->z) > 0.8) return Session::AngleBinding{Axis::Yaw, std::copysign(1.0, axis->z)};
                if (std::abs(across) > 0.8) return Session::AngleBinding{Axis::Pitch, std::copysign(1.0, across)};
                return std::nullopt;
            };
            const auto first = probe(true);
            const auto second = probe(false);
            if (first && second && first->axis != second->axis) {
                session.angles = {*first, *second};
                session.nativeAnglesValid = true;
            }
            return true;
        }

        void UpdateRenderView(RE::TESCameraState& state)
        {
            if (state.id != Mode::kThirdPerson && !(session.seat && state.id == Mode::kFurniture)) {
                renderViewState.store(nullptr);
                RenderCamera::Clear(true);
                return;
            }
            if (!session.view) {
                const auto* camera = RE::Main::WorldRootCamera();
                const auto rendered = DragView::ReadCamera();
                if (!camera || !rendered) return;
                const RE::NiQuaternion native{camera->world.rotate};
                const auto rotation = Unit(Quaternion{native.x, native.y, native.z, native.w});
                if (!rotation) return;
                const auto& eye = rendered->aim.origin;
                const auto right = Unit(Cross(rendered->aim.direction, Vec3{0, 0, 1})).value_or(rendered->right);
                session.view = Viewpoint{{float(eye.x), float(eye.y), float(eye.z)}, *rotation, rendered->aim.direction, right, {}};
                SKSE::log::info("Posing render view active in camera state {}", static_cast<unsigned>(state.id));
            }
            renderViewState.store(&state);
            RenderCamera::Publish(*session.view);
        }

    }

    bool Install()
    {
        GameplayInput<RE::MovementHandler>::Install();
        GameplayInput<RE::LookHandler>::Install();
        GameplayInput<RE::SprintHandler>::Install();
        GameplayInput<RE::ReadyWeaponHandler>::Install();
        GameplayInput<RE::AutoMoveHandler>::Install();
        GameplayInput<RE::ToggleRunHandler>::Install();
        GameplayInput<RE::ActivateHandler>::Install();
        GameplayInput<RE::JumpHandler>::Install();
        GameplayInput<RE::ShoutHandler>::Install();
        GameplayInput<RE::AttackBlockHandler>::Install();
        GameplayInput<RE::RunHandler>::Install();
        GameplayInput<RE::SneakHandler>::Install();
        GameplayInput<RE::TogglePOVHandler>::Install();
        REL::Relocation<std::uintptr_t> table{RE::VTABLE_FreeCameraState[0]};
        const bool freeUpdate = FreeUpdate::original.InstallVirtual(table, 3, FreeUpdate::Thunk);
        const bool freeRotation = FreeRotation::original.InstallVirtual(table, 4, FreeRotation::Thunk);
        REL::Relocation<std::uintptr_t> inputTable{RE::VTABLE_FreeCameraState[1]};
        const bool freeInput = FreeInput::original.InstallVirtual(inputTable, 1, FreeInput::Thunk);
        REL::Relocation<std::uintptr_t> renderCameraTable{RE::VTABLE_NiCamera[0]};
        const bool renderCamera = RenderCamera::original.InstallVirtual(renderCameraTable, 0x30, RenderCamera::Thunk);
        REL::Relocation<std::uintptr_t> thirdPersonInputTable{RE::VTABLE_ThirdPersonState[1]};
        const bool renderCameraInput = RenderCameraInput::original.InstallVirtual(thirdPersonInputTable, 1, RenderCameraInput::Thunk);
        const bool installed = freeUpdate && freeRotation && freeInput && renderCamera && renderCameraInput;
        if (!installed)
            SKSE::log::warn("One or more posing camera hooks could not be installed");
        return installed;
    }

    void Begin(RE::PlayerCharacter& player, bool seated)
    {
        auto* camera = RE::PlayerCamera::GetSingleton();
        if (!camera || !camera->currentState) return;
        const auto entry = camera->currentState->id;
        if (!FollowCamera(entry, seated) && entry != Mode::kFree) return;
        session = {};
        session.camera = camera;
        session.player = &player;
        session.cell = player.GetParentCell();
        session.root = player.Get3D(false);
        if (seated) session.seat = player.GetOccupiedFurniture();
        session.world = world.load();
        session.entry = session.last = entry;
        if (entry == Mode::kFree) {
            const auto* free = static_cast<RE::FreeCameraState*>(camera->currentState.get());
            session.entryFreePosition = free->translation;
            session.entryFreeAngles = free->rotation;
            if (const auto* map = RE::ControlMap::GetSingleton()) session.entryContext = HasFreeInput(*map);
        }
        if (const auto* sensitivity = RE::GetINISetting("fMouseHeadingSensitivity:Controls")) {
            const auto value = sensitivity->GetFloat();
            if (std::isfinite(value) && value > 0) session.radiansPerCount *= std::clamp(value / 0.0125, 0.1, 8.0);
        }
        if (const auto* inverted = RE::GetINISetting("bInvertYValues:Controls"); inverted && inverted->GetBool())
            session.verticalSign = -1;
        auto* controls = RE::PlayerControls::GetSingleton();
        if (controls) {
            controls->data.moveInputVec = {};
            controls->data.lookInputVec = {};
            controls->data.prevLookVec = {};
        }
        Update();
    }

    void Update()
    {
        auto* camera = RE::PlayerCamera::GetSingleton();
        if (!camera || session.camera != camera || session.world != world.load() || !camera->currentState) return;
        const auto current = camera->currentState->id;
        if (session.seat && (session.entry != Mode::kFree || current != Mode::kFree)) {
            protectedState.store(nullptr);
            UpdateRenderView(*camera->currentState);
            return;
        }
        if (renderViewState.exchange(nullptr)) RenderCamera::Clear(true);
        auto* free = static_cast<RE::FreeCameraState*>(camera->GetRuntimeData().cameraStates[Mode::kFree].get());
        if (!free) return;
        if (current != session.last) {
            SKSE::log::info("Posing camera state {} -> {}", static_cast<unsigned>(session.last), static_cast<unsigned>(current));
            session.last = current;
        }
        const auto now = Clock::now();
        if (protectedState.load() == free && current == Mode::kFree) {
            if (now - session.attempted > std::chrono::seconds(1)) session.attempts = 0;
            return;
        }
        protectedState.store(nullptr);
        if (current != Mode::kFree && (!FollowCamera(current, false) || session.attempts >= 3 ||
            now - session.attempted < std::chrono::milliseconds(250))) return;

        const auto rendered = session.view ? std::nullopt : DragView::ReadCamera();
        if (!rendered && !session.view) return;
        if (current != Mode::kFree) {
            ++session.attempts;
            session.attempted = now;
            if (!session.acquired) {
                camera->ToggleFreeCameraMode(false);
                session.nativeEntry = camera->IsInFreeCameraMode();
            }
            if (!camera->IsInFreeCameraMode()) camera->SetState(free);
            if (!camera->IsInFreeCameraMode()) return;
            session.acquired = true;
        }
        // The detached view still uses the engine's free camera state. Its input
        // context is unnecessary because the editor consumes filtered events.
        RemoveFreeInput();
        if (!session.view && !SeedView(*free, *rendered)) return;
        free->translation = session.view->position;
        free->rotation = session.view->angles;
        protectedState.store(free);
        SKSE::log::info("Posing camera detached with gameplay input context");
    }

    void Fly(const FlightInput& input, double dt)
    {
        auto* free = protectedState.load();
        const auto* state = session.camera ? session.camera->currentState.get() : nullptr;
        if (!state || (state != free && state != renderViewState.load()) || !session.view || input.rotating) return;
        auto& view = *session.view;
        const bool looking = input.mouseX != 0 || input.mouseY != 0;
        if (looking) {
            const auto yaw = -input.mouseX * session.radiansPerCount;
            const auto pitch = -input.mouseY * session.radiansPerCount * session.verticalSign;
            const auto elevation = std::asin(std::clamp(view.forward.z, -1.0, 1.0));
            const auto turn = CameraLookDelta(view.forward, view.right, yaw, pitch);
            view.rotation = *Unit(Multiply(turn, view.rotation));
            view.forward = *Unit(*Rotate(turn, view.forward));
            view.right = *Unit(*Rotate(turn, view.right));
            if (session.nativeAnglesValid) {
                const auto actualPitch = std::asin(std::clamp(view.forward.z, -1.0, 1.0)) - elevation;
                const auto delta = [&](const Session::AngleBinding& bind) {
                    return float(bind.sign * (bind.axis == Session::AngleAxis::Yaw ? yaw : actualPitch));
                };
                constexpr float turn = 2 * std::numbers::pi_v<float>;
                view.angles.x = std::remainder(view.angles.x + delta(session.angles[0]), turn);
                view.angles.y = std::remainder(view.angles.y + delta(session.angles[1]), turn);
            }
        }
        const auto movement = view.forward * input.forward + view.right * input.right;
        const auto direction = Length(movement) > 1 ? *Unit(movement) : movement;
        const auto move = direction * (dt * (input.slow ? 40 : 315));
        view.position += {float(move.x), float(move.y), float(move.z)};
        if (free) {
            free->translation = view.position;
            free->rotation = view.angles;
        } else if (looking || input.forward || input.right) {
            RenderCamera::Publish(view);
            if (!session.translationReported && (input.forward || input.right) && dt > 0) {
                const auto* camera = RE::Main::WorldRootCamera();
                if (camera) {
                    const auto error = camera->world.translate - view.position;
                    SKSE::log::debug("Posing render camera translation: forward={} right={} view error={:.3f}",
                        input.forward, input.right, error.Length());
                    session.translationReported = true;
                }
            }
        }
        if (!session.flightReported && (looking || input.forward || input.right)) {
            session.flightReported = true;
            SKSE::log::debug("Posing camera received flight input: look={} forward={} right={} nativeAngles={}",
                looking, input.forward, input.right, session.nativeAnglesValid);
        }
    }

    void End(RE::PlayerCharacter* player)
    {
        const bool hadRenderView = renderViewState.exchange(nullptr) != nullptr;
        RenderCamera::Clear(true);
        if (hadRenderView) SKSE::log::debug("Posing render view released");
        protectedState.store(nullptr);
        if (!session.camera) return;
        const auto previous = std::exchange(session, {});
        auto* camera = RE::PlayerCamera::GetSingleton();
        if (!camera || camera != previous.camera || previous.world != world.load() ||
            previous.player != player || !player || !camera->currentState) return;
        if (previous.entry == Mode::kFree) {
            if (!camera->IsInFreeCameraMode()) return;
            auto* free = static_cast<RE::FreeCameraState*>(camera->currentState.get());
            free->translation = previous.entryFreePosition;
            free->rotation = previous.entryFreeAngles;
            if (auto* map = RE::ControlMap::GetSingleton(); previous.entryContext &&
                map && !HasFreeInput(*map)) map->PushInputContext(kTFC);
            return;
        }
        if (!previous.acquired || !camera->IsInFreeCameraMode()) return;
        const bool sameScene = player->GetParentCell() == previous.cell && player->Get3D(false) == previous.root;
        const auto mode = sameScene ? previous.entry : Mode::kThirdPerson;
        if (previous.nativeEntry) camera->ToggleFreeCameraMode(false);
        if (camera->IsInFreeCameraMode() || (camera->currentState && FollowCamera(camera->currentState->id, false) &&
            camera->currentState->id != mode)) {
            if (auto* back = camera->GetRuntimeData().cameraStates[mode].get()) camera->SetState(back);
        }
        SKSE::log::info("Posing camera released, state={}", camera->currentState ? static_cast<int>(camera->currentState->id) : -1);
    }

    void WorldChanged()
    {
        renderViewState.store(nullptr);
        RenderCamera::Clear(false);
        protectedState.store(nullptr);
        world.fetch_add(1);
    }
}
