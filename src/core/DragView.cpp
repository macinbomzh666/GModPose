#include <RE/Skyrim.h>
#include <SKSE/SKSE.h>
#include <Windows.h>
#include <d3d11.h>
#include <wrl/client.h>
#include <imgui.h>
#include <backends/imgui_impl_dx11.h>

#include "DragView.h"
#include "Editor.h"
#include "HookOriginal.h"
#include "PoseSettings.h"
#include <atomic>
#include <bit>
#include <memory>
#include <mutex>

namespace GModPose::DragView
{
    namespace
    {
        Frame frame;
        std::mutex frameMutex;
        std::atomic_bool installed{false}, renderFailed{false};

        using Microsoft::WRL::ComPtr;
        template <class Shader, auto GetShader, auto SetShader>
        struct SavedShader
        {
            ID3D11DeviceContext& context;
            ComPtr<Shader> shader;
            std::array<ID3D11ClassInstance*, D3D11_SHADER_MAX_INTERFACES> instances{};
            UINT count = static_cast<UINT>(instances.size());

            explicit SavedShader(ID3D11DeviceContext& immediate) : context(immediate)
            {
                (context.*GetShader)(shader.GetAddressOf(), instances.data(), &count);
            }

            ~SavedShader()
            {
                (context.*SetShader)(shader.Get(), instances.data(), count);
                for (auto* instance : instances)
                    if (instance) instance->Release();
            }
        };

        struct OverlayPipeline
        {
            ID3D11DeviceContext& context;
            std::array<ID3D11RenderTargetView*, D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT> targets{};
            ComPtr<ID3D11DepthStencilView> depth;
            std::array<ID3D11UnorderedAccessView*, D3D11_1_UAV_SLOT_COUNT> uavs{};
            UINT targetCount = 0;
            UINT uavCount;
            // The ImGui backend clears these stages without saving them.
            SavedShader<ID3D11HullShader, &ID3D11DeviceContext::HSGetShader, &ID3D11DeviceContext::HSSetShader> hull;
            SavedShader<ID3D11DomainShader, &ID3D11DeviceContext::DSGetShader, &ID3D11DeviceContext::DSSetShader> domain;
            SavedShader<ID3D11ComputeShader, &ID3D11DeviceContext::CSGetShader, &ID3D11DeviceContext::CSSetShader> compute;

            OverlayPipeline(ID3D11DeviceContext& immediate, D3D_FEATURE_LEVEL level) :
                context(immediate),
                uavCount(level >= D3D_FEATURE_LEVEL_11_1 ? D3D11_1_UAV_SLOT_COUNT :
                    level >= D3D_FEATURE_LEVEL_11_0 ? D3D11_PS_CS_UAV_REGISTER_COUNT : 0),
                hull(context), domain(context), compute(context)
            {
                context.OMGetRenderTargets(static_cast<UINT>(targets.size()), targets.data(), depth.GetAddressOf());
                for (UINT i = 0; i < targets.size(); ++i)
                    if (targets[i]) targetCount = i + 1;
                if (targetCount < uavCount)
                    context.OMGetRenderTargetsAndUnorderedAccessViews(0, nullptr, nullptr,
                        targetCount, uavCount - targetCount, uavs.data() + targetCount);
            }

            ~OverlayPipeline()
            {
                if (std::ranges::any_of(uavs, [](auto* view) { return view != nullptr; })) {
                    // Preserve the game's append and consume counters.
                    context.OMSetRenderTargetsAndUnorderedAccessViews(targetCount, targets.data(), depth.Get(),
                        targetCount, uavCount - targetCount, uavs.data() + targetCount, nullptr);
                } else {
                    context.OMSetRenderTargets(targetCount, targets.data(), depth.Get());
                }
                for (auto* target : targets)
                    if (target) target->Release();
                for (auto* uav : uavs)
                    if (uav) uav->Release();
            }
        };

        struct Color
        {
            float r, g, b, a;

            ImU32 Packed() const { return ImGui::ColorConvertFloat4ToU32({r, g, b, a}); }
        };

        struct OverlayGeometry
        {
            ImDrawList& drawList;
            const ImVec2 white = ImGui::GetFontTexUvWhitePixel();
            const float framebufferScale;

            ImVec2 Point(Vec3 p) const
            {
                return {static_cast<float>(p.x) / framebufferScale, static_cast<float>(p.y) / framebufferScale};
            }

            void Stroke(Vec3 from, Vec3 to, double width, Color color)
            {
                const auto normal = Unit(Vec3{from.y - to.y, to.x - from.x, 0});
                if (!normal) return;
                const auto side = *normal * (width * 0.5);
                drawList.PrimReserve(6, 4);
                drawList.PrimQuadUV(Point(from + side), Point(to + side), Point(to - side), Point(from - side),
                    white, white, white, white, color.Packed());
            }

            void Disc(Vec3 center, double radius, Vec3 highlight, Color inner, Color rim)
            {
                constexpr int segments = 36;
                const auto middle = Point(center + highlight);
                const auto innerColor = inner.Packed();
                const auto rimColor = rim.Packed();
                auto previous = Point(center + Vec3{radius, 0, 0});
                drawList.PrimReserve(3 * segments, 3 * segments);
                for (int i = 1; i <= segments; ++i) {
                    const double angle = 2 * std::numbers::pi * i / segments;
                    const auto next = Point(center + Vec3{radius * std::cos(angle), radius * std::sin(angle), 0});
                    drawList.PrimVtx(middle, white, innerColor);
                    drawList.PrimVtx(previous, white, rimColor);
                    drawList.PrimVtx(next, white, rimColor);
                    previous = next;
                }
            }

            void DrawMiddlePulse(Vec3 from, Vec3 to, float strength, double scale)
            {
                const auto delta = to - from;
                const auto across = Unit(Vec3{-delta.y, delta.x, 0});
                if (!across) return;

                constexpr std::array jagged{0.0, -5.0, 8.0, -11.0, 7.0, -6.0, 5.0, -4.0, 0.0};
                std::array<Vec3, jagged.size()> points;
                for (std::size_t i = 0; i < points.size(); ++i)
                    points[i] = from + delta * (double(i) / (points.size() - 1)) + *across * (jagged[i] * scale);
                const auto forkEnd = points[5] + delta * .09 + *across * (19 * scale);
                const auto forkKink = points[4] + delta * .07 + *across * (9 * scale);
                const auto stroke = [&](double width, Color color, bool branches) {
                    for (std::size_t i = 1; i < points.size(); ++i)
                        Stroke(points[i - 1], points[i], width * scale, color);
                    if (branches) {
                        Stroke(points[4], forkKink, width * .62 * scale, color);
                        Stroke(forkKink, forkEnd, width * .45 * scale, color);
                    }
                };

                stroke(10, {0.14F, 0.46F, 1.0F, strength * .24F}, false);
                stroke(4.5, {0.34F, 0.74F, 1.0F, strength * .78F}, true);
                stroke(1.65, {0.88F, 0.97F, 1.0F, strength}, true);
                Disc(from, 14 * scale, {}, {0.37F, 0.75F, 1.0F, strength * .7F}, {0.17F, 0.40F, 1.0F, 0});
                Disc(from, 4 * scale, {}, {0.94F, 1.0F, 1.0F, strength}, {0.43F, 0.77F, 1.0F, strength * .65F});
            }

            void Draw(const Frame& view, float width, float height, float pulseStrength)
            {
                const auto pixels = [width, height](Vec3 p) {
                    return Vec3{(1 + p.x) * width * 0.5, (1 - p.y) * height * 0.5, 0};
                };
                const double scale = std::clamp(height / 1080.0, 0.75, 2.0);
                if (view.visible && view.drawWorld && !view.settings && !view.finger && !view.face && !view.body) {
                    const auto glow = view.rotating ? Color{1.0F, 0.53F, 0.12F, 0.30F} :
                        Color{0.2F, 0.65F, 1.0F, 0.25F};
                    const auto beam = view.rotating ? Color{1.0F, 0.78F, 0.30F, 0.95F} :
                        Color{0.69F, 0.94F, 1.0F, 0.93F};
                    const auto drawCurve = [&](double strokeWidth, Color color) {
                        for (std::size_t i = 0; i < view.count; ++i)
                            Stroke(pixels(view.tether[i].start), pixels(view.tether[i].end), strokeWidth * scale, color);
                    };
                    drawCurve(8, glow);
                    drawCurve(2.5, beam);
                    if (view.orb) {
                        const auto p = pixels(*view.orb);
                        const auto shadow = view.rotating ? Color{0.54F, 0.22F, 0.03F, 1} :
                            Color{0.025F, 0.23F, 0.47F, 1};
                        Disc(p, 19 * scale, {}, beam, {glow.r, glow.g, glow.b, 0});
                        Disc(p, 8.5 * scale, Vec3{-2.6, -3.0, 0} * scale, {1, 1, 1, 1}, shadow);
                        Disc(p + Vec3{-2.6, -3.0, 0} * scale, 2.2 * scale, {}, {1, 1, 1, 1}, beam);
                    }
                    const Color outline{0.015F, 0.035F, 0.05F, 0.95F};
                    const Color idle{0.94F, 0.98F, 1.0F, 1.0F};
                    const Color hover{0.18F, 1.0F, 0.90F, 1.0F};
                    const auto outlined = [&](Vec3 a, Vec3 b, Color tint) {
                        Stroke(a, b, 4.5 * scale, outline);
                        Stroke(a, b, 2.0 * scale, tint);
                    };
                    const auto selected = view.held ? view.held : view.hover;
                    const auto tint = view.limited || view.rotating ? beam : selected ? hover : idle;
                    const auto marker = [&](std::size_t i, bool highlight) {
                        const auto p = pixels(*view.markers[i]);
                        const double r = (highlight ? 8 : 5) * scale;
                        const auto color = highlight ? tint : Color{0.48F, 0.8F, 1.0F, 0.95F};
                        const auto bit = std::uint64_t{1} << i;
                        if (view.triangleHandles & bit) {
                            const auto top = p + Vec3{0, -r, 0};
                            const auto left = p + Vec3{-.866 * r, .5 * r, 0};
                            const auto right = p + Vec3{.866 * r, .5 * r, 0};
                            outlined(top, left, color);
                            outlined(left, right, color);
                            outlined(right, top, color);
                            return;
                        }
                        if (view.circleHandles & bit) {
                            auto previous = p + Vec3{r, 0, 0};
                            for (int segment = 1; segment <= 24; ++segment) {
                                const double angle = 2 * std::numbers::pi * segment / 24;
                                const auto next = p + Vec3{std::cos(angle) * r, std::sin(angle) * r, 0};
                                outlined(previous, next, color);
                                previous = next;
                            }
                            return;
                        }
                        const Vec3 x{r, 0, 0}, y{0, r, 0};
                        outlined(p - x, p - y, color);
                        outlined(p - y, p + x, color);
                        outlined(p + x, p + y, color);
                        outlined(p + y, p - x, color);
                        if (view.plantedHandles & bit) {
                            const auto ground = p + Vec3{0, r + 4 * scale, 0};
                            outlined(ground - x * .9, ground + x * .9, color);
                        }
                    };
                    for (std::size_t i = 0; i < view.markers.size(); ++i)
                        if (view.markers[i] && selected != i) marker(i, false);
                    if (selected && *selected < view.markers.size() && view.markers[*selected]) marker(*selected, true);

                    // Four outlined ticks leave the center clear for the selected handle.
                    const auto center = pixels({});
                    if (view.mirror) {
                        for (const double x : {-7.0, 7.0})
                            outlined(center + Vec3{x, -13, 0} * scale, center + Vec3{x, 13, 0} * scale, tint);
                    } else {
                        for (const auto axis : {Vec3{1, 0, 0}, Vec3{-1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, -1, 0}})
                            outlined(center + axis * (9 * scale), center + axis * (18 * scale), tint);
                    }
                }
                if (view.middlePulse && pulseStrength > 0) {
                    const auto& span = view.middlePulse->span;
                    DrawMiddlePulse(pixels(span.start), pixels(span.end), pulseStrength, scale);
                }
            }
        };

        struct Renderer
        {
            ID3D11Device* device;
            ID3D11DeviceContext* context;
            D3D_FEATURE_LEVEL featureLevel;
            ImGuiContext* ui;

            Renderer(ID3D11Device* device, ID3D11DeviceContext* context) :
                device(device), context(context), featureLevel(device->GetFeatureLevel()),
                ui(PoseSettings::CreateUI(device, context))
            {}

            ~Renderer() { PoseSettings::DestroyUI(ui); }

            void Draw(const Frame& view, ID3D11RenderTargetView* target, float width, float height, float pulseStrength)
            {
                struct RestoreContext
                {
                    ImGuiContext* previous = ImGui::GetCurrentContext();
                    ~RestoreContext() { ImGui::SetCurrentContext(previous); }
                } restore;
                ImGui::SetCurrentContext(ui);
                PoseSettings::DrawUI(view, width, height, pulseStrength);

                OverlayPipeline pipeline(*context, featureLevel);
                context->OMSetRenderTargets(1, &target, nullptr);
                ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
            }
        };

        void DrawFrame()
        {
            if (renderFailed.load()) return;
            Frame view;
            float pulseStrength = 0;
            {
                std::lock_guard lock(frameMutex);
                if (frame.middlePulse) {
                    pulseStrength = frame.middlePulse->Intensity(std::chrono::steady_clock::now());
                    if (pulseStrength <= 0) frame.middlePulse.reset();
                }
                if (!frame.visible && pulseStrength <= 0) return;
                view = frame;
            }
            static std::string_view pending;
            const auto waiting = [](std::string_view reason) {
                if (pending != reason) SKSE::log::info("Posing reticle waiting for {}", reason);
                pending = reason;
            };
            auto* gameRenderer = RE::BSGraphics::Renderer::GetSingleton();
            if (!gameRenderer) return waiting("renderer");
            auto& data = gameRenderer->GetRuntimeData();
            const auto& window = data.renderWindows[0];
            if (!data.context || !data.forwarder) return waiting("D3D11 context");
            auto* target = data.renderTargets[RE::RENDER_TARGETS::kFRAMEBUFFER].RTV;
            if (!target) target = window.renderView;
            if (!target) return waiting("frame target");
            auto* rtv = reinterpret_cast<ID3D11RenderTargetView*>(target);
            ComPtr<ID3D11Resource> resource;
            ComPtr<ID3D11Texture2D> surface;
            rtv->GetResource(resource.GetAddressOf());
            if (!resource || FAILED(resource.As(&surface))) return waiting("frame texture");
            D3D11_TEXTURE2D_DESC desc{};
            surface->GetDesc(&desc);
            if (!desc.Width || !desc.Height) return waiting("nonempty frame texture");
            pending = {};
            auto* device = reinterpret_cast<ID3D11Device*>(data.forwarder);
            auto* context = reinterpret_cast<ID3D11DeviceContext*>(data.context);
            static std::unique_ptr<Renderer> renderer;
            if (!renderer || renderer->device != device || renderer->context != context) {
                renderer = std::make_unique<Renderer>(device, context);
                if (!renderer->ui) {
                    renderFailed.store(true);
                    SKSE::log::warn("Posing renderer unavailable: D3D11 or ImGui initialization failed");
                    return;
                }
                SKSE::log::info("Posing overlay initialized on Skyrim's D3D11 context ({}x{})", desc.Width, desc.Height);
            }
            renderer->Draw(view, rtv, static_cast<float>(desc.Width), static_cast<float>(desc.Height), pulseStrength);
        }

        struct PresentHook
        {
            // Timer call at the start of Renderer::End, before swap-chain presentation.
            static void Thunk(std::uint32_t timer)
            {
                original(timer);
                try {
                    DrawFrame();
                } catch (const std::exception& error) {
                    renderFailed.store(true);
                    SKSE::log::warn("Mouse tether drawing stopped: {}", error.what());
                }
            }
            static inline HookOriginal<decltype(Thunk)> original;
        };
    }

    void DrawOverlay(const Frame& view, float width, float height, float framebufferScale, float pulseStrength)
    {
        OverlayGeometry geometry{.drawList = *ImGui::GetBackgroundDrawList(), .framebufferScale = framebufferScale};
        geometry.Draw(view, width, height, pulseStrength);
    }

    bool Install()
    {
        if (installed.load()) return true;
        const REL::Relocation<std::uintptr_t> site{RELOCATION_ID(75461, 77246), 0x9};
        if (!PresentHook::original.InstallCall(site.address(), PresentHook::Thunk)) {
            SKSE::log::warn("Mouse tether unavailable: expected CALL rel32 in Renderer::End");
            return false;
        }
        installed.store(true);
        SKSE::log::info("Installed local posing reticle and tether renderer");
        return true;
    }

    bool Available() { return installed.load() && !renderFailed.load(); }

    const char* InputBlockReason()
    {
        auto* ui = RE::UI::GetSingleton();
        auto* control = RE::ControlMap::GetSingleton();
        auto* main = RE::Main::GetSingleton();
        if (!main || (!main->GetRuntimeData().gameActive &&
            (!main->wnd || ::GetForegroundWindow() != reinterpret_cast<HWND>(main->wnd)))) return "window inactive";
        if (!ui || ui->GameIsPaused() || ui->IsApplicationMenuOpen() || ui->IsMenuOpen("Console")) return "menu";
        if (!control || control->GetRuntimeData().textEntryCount > int(Runtime::OwnsTextInput(control))) return "text entry";
        const auto& contexts = control->GetRuntimeData().contextPriorityStack;
        if (!contexts.empty() && contexts.back() != RE::ControlMap::InputContextID::kGameplay &&
            contexts.back() != RE::ControlMap::InputContextID::kTFCMode) return "menu";
        return nullptr;
    }

    std::optional<Camera> ReadCamera()
    {
        auto* renderer = RE::BSGraphics::Renderer::GetSingleton();
        auto* camera = RE::Main::WorldRootCamera();
        if (!renderer || !camera) return std::nullopt;
        const auto& window = renderer->GetRuntimeData().renderWindows[0];
        REX::W32::DXGI_SWAP_CHAIN_DESC desc{};
        const bool swapSize = window.swapChain && window.swapChain->GetDesc(&desc) >= 0 &&
                              desc.bufferDesc.width && desc.bufferDesc.height;
        if (!swapSize && (window.windowWidth <= 0 || window.windowHeight <= 0)) return std::nullopt;

        const auto& matrix = camera->GetRuntimeData().worldToCam;
        const auto [left, rightEdge, top, bottom] =
            std::bit_cast<std::array<float, 4>>(camera->GetRuntimeData2().port);
        Camera view{};
        view.width = swapSize ? static_cast<double>(desc.bufferDesc.width) : window.windowWidth;
        view.height = swapSize ? static_cast<double>(desc.bufferDesc.height) : window.windowHeight;
        view.nearPlane = camera->GetNearPlane();
        // Fold the camera's viewport into the homogeneous coordinates so line
        // clipping operates on the actual displayed rectangle.
        for (std::size_t i = 0; i < view.w.size(); ++i) {
            view.w[i] = matrix[3][i];
            view.x[i] = (rightEdge - left) * matrix[0][i] + (left + rightEdge - 1) * view.w[i];
            view.y[i] = (top - bottom) * matrix[1][i] + (top + bottom - 1) * view.w[i];
        }
        const auto& pos = camera->world.translate;
        if (!view.SetAim({pos.x, pos.y, pos.z})) return std::nullopt;
        return view;
    }

    void Publish(Frame next)
    {
        std::lock_guard lock(frameMutex);
        if (next.drawWorld && !next.middlePulse) next.middlePulse = frame.middlePulse;
        frame = std::move(next);
    }

    void Clear()
    {
        std::lock_guard lock(frameMutex);
        frame = {};
    }
}
