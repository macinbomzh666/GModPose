#include <SKSE/SKSE.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <cstdint>
#include "core/Editor.h"
#include "core/FacePanel.h"
#include "Version.h"

SKSEPluginInfo(
    .Version = {GMODPOSE_VERSION_NUMBERS},
    .Name = "GModPose",
    .Author = "Macinbomzh",
    .StructCompatibility = SKSE::StructCompatibility::Independent,
    .RuntimeCompatibility = SKSE::VersionIndependence::AddressLibrary
);

// Optional in-process query for other SKSE plugins. Bit 0 signals a captured
// editor pose. Bit 1 signals that the editor owns the player's FaceGen values.
extern "C" __declspec(dllexport) std::uint32_t GModPose_GetEditorStateV1() noexcept
{
    if (!GModPose::Runtime::Running()) return 0;
    return GModPose::FaceActor::OwnsFace() ? 3U : 1U;
}

SKSE_PLUGIN_LOAD(const SKSE::LoadInterface* skse)
{
    // The editor relies on native call boundaries checked against these runtimes.
    if (skse->IsEditor() || (skse->RuntimeVersion() != SKSE::RUNTIME_SSE_1_5_97 &&
                            skse->RuntimeVersion() != SKSE::RUNTIME_SSE_1_6_1170 &&
                            skse->RuntimeVersion() != SKSE::RUNTIME_SSE_1_7_104)) return false;
    SKSE::Init(skse, {.log = false, .trampoline = true, .trampolineSize = 160});
    const auto directory = SKSE::log::log_directory();
    if (!directory) return false;
    try {
        auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            (*directory / "GModPose.log").string(), 2 * 1024 * 1024, 2, true);
        auto logger = std::make_shared<spdlog::logger>("GModPose", sink);
        logger->set_level(spdlog::level::debug);
        logger->flush_on(spdlog::level::debug);
        spdlog::set_default_logger(logger);
    } catch (const std::exception&) {
        return false;
    }
    SKSE::log::info("GModPose IK editor " GMODPOSE_VERSION_TEXT "; Skyrim={}", skse->RuntimeVersion().string());
    if (!GModPose::Runtime::InstallEarlyUpdate()) return false;
    const auto* messaging = SKSE::GetMessagingInterface();
    return messaging && messaging->RegisterListener([](SKSE::MessagingInterface::Message* message) {
        switch (message->type) {
        case SKSE::MessagingInterface::kDataLoaded:
            if (!GModPose::Runtime::Install())
                SKSE::log::error("Editor unavailable: installation checks failed");
            break;
        case SKSE::MessagingInterface::kPreLoadGame:
            GModPose::Runtime::SetLoading(true);
            break;
        case SKSE::MessagingInterface::kPostLoadGame:
            GModPose::Runtime::SetLoading(message->data == nullptr);
            break;
        case SKSE::MessagingInterface::kNewGame:
            GModPose::Runtime::SetLoading(false);
            break;
        case SKSE::MessagingInterface::kSaveGame:
            GModPose::Runtime::Invalidate();
            break;
        default:
            break;
        }
    });
}
