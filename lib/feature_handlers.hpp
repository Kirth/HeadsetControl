#pragma once

#include "capability_descriptors.hpp"
#include "device.hpp"
#include "devices/hid_device.hpp"
#include "result_types.hpp"

#include <array>
#include <functional>
#include <optional>
#include <string>
#include <variant>

namespace headsetcontrol {

/**
 * @brief Unified feature output that can represent any feature result
 */
struct FeatureOutput {
    int value = 0; // Primary result value
    std::string message; // Human-readable message
    std::optional<BatteryResult> battery; // Extended battery info
    std::optional<ChatmixResult> chatmix; // Extended chatmix info
    std::optional<SidetoneResult> sidetone; // Extended sidetone info

    static FeatureOutput success(int val, std::string msg = "")
    {
        return { .value = val, .message = std::move(msg) };
    }

    static FeatureOutput fromBattery(const BatteryResult& b)
    {
        return {
            .value   = b.level_percent,
            .message = "",
            .battery = b
        };
    }

    static FeatureOutput fromChatmix(const ChatmixResult& c)
    {
        return {
            .value   = c.level,
            .message = std::format("Chat-Mix: {}", c.level),
            .chatmix = c
        };
    }

    static FeatureOutput fromSidetone(const SidetoneResult& s)
    {
        return {
            .value    = s.current_level,
            .message  = "",
            .sidetone = s
        };
    }
};

/**
 * @brief Handler function signature for feature execution
 */
using FeatureHandler = std::function<Result<FeatureOutput>(
    HIDDevice* device,
    hid_device* handle,
    const FeatureParam& param)>;

// Dispatch table mapping capabilities to handlers
class FeatureHandlerRegistry {
public:
    static FeatureHandlerRegistry& instance()
    {
        static FeatureHandlerRegistry registry;
        return registry;
    }

    /**
     * @brief Register a handler for a capability
     */
    void registerHandler(capabilities cap, FeatureHandler handler)
    {
        handlers_[static_cast<size_t>(cap)] = std::move(handler);
    }

    /**
     * @brief Register a handler that reads back the current value of a capability
     */
    void registerReadHandler(capabilities cap, FeatureHandler handler)
    {
        read_handlers_[static_cast<size_t>(cap)] = std::move(handler);
    }

    /**
     * @brief Check if a handler is registered for a capability
     */
    [[nodiscard]] bool hasHandler(capabilities cap) const
    {
        return handlers_[static_cast<size_t>(cap)] != nullptr;
    }

    /**
     * @brief Execute a feature
     * @param cap Capability to execute
     * @param device Device implementation
     * @param handle HID device handle (may be nullptr for test devices)
     * @param param Feature parameter
     * @return Result with feature output or error
     */
    [[nodiscard]] Result<FeatureOutput> execute(
        capabilities cap,
        HIDDevice* device,
        hid_device* handle,
        const FeatureParam& param) const
    {
        const auto& handler = handlers_[static_cast<size_t>(cap)];
        if (!handler) {
            return DeviceError::notSupported(
                std::format("No handler registered for {}", capability_to_string(cap)));
        }
        return handler(device, handle, param);
    }

    /**
     * @brief Read back the current value of a capability
     *
     * The output value uses the encoding the capability's setter takes, e.g. the
     * ANC mode or the inactive time in minutes.
     */
    [[nodiscard]] Result<FeatureOutput> executeRead(
        capabilities cap,
        HIDDevice* device,
        hid_device* handle) const
    {
        const auto& handler = read_handlers_[static_cast<size_t>(cap)];
        if (!handler) {
            return DeviceError::notSupported(
                std::format("Reading {} is not supported", capability_to_string(cap)));
        }
        return handler(device, handle, std::monostate {});
    }

private:
    FeatureHandlerRegistry()
    {
        registerAllHandlers();
        registerAllReadHandlers();
    }

    void registerAllHandlers();
    void registerAllReadHandlers();

    std::array<FeatureHandler, NUM_CAPABILITIES> handlers_ {};
    std::array<FeatureHandler, NUM_CAPABILITIES> read_handlers_ {};
};

/**
 * @brief Validate a feature parameter against its descriptor
 * @param cap Capability to validate for
 * @param param Parameter to validate
 * @return Error message if invalid, nullopt if valid
 */
[[nodiscard]] inline std::optional<std::string> validateFeatureParam(
    capabilities cap,
    const FeatureParam& param)
{
    const auto& desc = getCapabilityDescriptor(cap);

    // Info features should have no parameter (monostate)
    if (desc.isInfoFeature()) {
        if (!std::holds_alternative<std::monostate>(param) && !std::holds_alternative<int>(param)) {
            return std::format("{} doesn't take a parameter", desc.name);
        }
        return std::nullopt; // Valid
    }

    // Action features need a parameter
    if (std::holds_alternative<std::monostate>(param)) {
        // Allow monostate for features that might have default behavior
        return std::nullopt;
    }

    // Validate integer range if applicable
    if (auto* val = std::get_if<int>(&param)) {
        if (desc.min_value && *val < *desc.min_value) {
            return std::format("{} must be >= {} (got {})",
                desc.name, *desc.min_value, *val);
        }
        if (desc.max_value && *val > *desc.max_value) {
            return std::format("{} must be <= {} (got {})",
                desc.name, *desc.max_value, *val);
        }
    }

    return std::nullopt; // Valid
}

// ============================================================================
// Handler Registration Helpers
// ============================================================================

namespace detail {

    // Helper to get int from FeatureParam
    inline int getInt(const FeatureParam& p)
    {
        return std::get<int>(p);
    }

    inline uint8_t getUint8(const FeatureParam& p)
    {
        return static_cast<uint8_t>(std::get<int>(p));
    }

    inline bool getBool(const FeatureParam& p)
    {
        return std::get<int>(p) != 0;
    }

    inline const EqualizerSettings& getEqualizer(const FeatureParam& p)
    {
        return std::get<EqualizerSettings>(p);
    }

    inline const ParametricEqualizerSettings& getParametricEq(const FeatureParam& p)
    {
        return std::get<ParametricEqualizerSettings>(p);
    }

    inline std::string_view ancModeName(uint8_t mode)
    {
        switch (mode) {
        case 0:
            return "off";
        case 1:
            return "anc";
        case 2:
            return "ambient";
        case 3:
            return "last"; // ANC startup mode only: mode at power off
        default:
            return "unknown";
        }
    }

    inline const AncButtonModes& getAncButtonModes(const FeatureParam& p)
    {
        return std::get<AncButtonModes>(p);
    }

} // namespace detail

// ============================================================================
// Handler Registration
// ============================================================================

inline void FeatureHandlerRegistry::registerAllHandlers()
{
    using namespace detail;

    // CAP_SIDETONE
    registerHandler(CAP_SIDETONE, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setSidetone(h, getUint8(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::fromSidetone(r.value());
    });

    // CAP_BATTERY_STATUS
    registerHandler(CAP_BATTERY_STATUS, [](HIDDevice* dev, hid_device* h, const FeatureParam&) -> Result<FeatureOutput> {
        auto r = dev->getBattery(h);
        if (r.hasError())
            return r.error();
        return FeatureOutput::fromBattery(r.value());
    });

    // CAP_NOTIFICATION_SOUND
    registerHandler(CAP_NOTIFICATION_SOUND, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->notificationSound(h, getUint8(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->sound_id);
    });

    // CAP_LIGHTS
    registerHandler(CAP_LIGHTS, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setLights(h, getBool(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->enabled ? 1 : 0);
    });

    // CAP_INACTIVE_TIME
    registerHandler(CAP_INACTIVE_TIME, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setInactiveTime(h, getUint8(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->minutes);
    });

    // CAP_CHATMIX_STATUS
    registerHandler(CAP_CHATMIX_STATUS, [](HIDDevice* dev, hid_device* h, const FeatureParam&) -> Result<FeatureOutput> {
        auto r = dev->getChatmix(h);
        if (r.hasError())
            return r.error();
        return FeatureOutput::fromChatmix(r.value());
    });

    // CAP_VOICE_PROMPTS
    registerHandler(CAP_VOICE_PROMPTS, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setVoicePrompts(h, getBool(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->enabled ? 1 : 0);
    });

    // CAP_ROTATE_TO_MUTE
    registerHandler(CAP_ROTATE_TO_MUTE, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setRotateToMute(h, getBool(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->enabled ? 1 : 0);
    });

    // CAP_EQUALIZER_PRESET
    registerHandler(CAP_EQUALIZER_PRESET, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setEqualizerPreset(h, getUint8(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->preset);
    });

    // CAP_EQUALIZER
    registerHandler(CAP_EQUALIZER, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setEqualizer(h, getEqualizer(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(0);
    });

    // CAP_PARAMETRIC_EQUALIZER
    registerHandler(CAP_PARAMETRIC_EQUALIZER, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setParametricEqualizer(h, getParametricEq(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(0);
    });

    // CAP_MICROPHONE_MUTE_LED_BRIGHTNESS
    registerHandler(CAP_MICROPHONE_MUTE_LED_BRIGHTNESS, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setMicMuteLedBrightness(h, getUint8(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->brightness);
    });

    // CAP_MICROPHONE_VOLUME
    registerHandler(CAP_MICROPHONE_VOLUME, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setMicVolume(h, getUint8(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->volume);
    });

    // CAP_VOLUME_LIMITER
    registerHandler(CAP_VOLUME_LIMITER, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setVolumeLimiter(h, getBool(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->enabled ? 1 : 0);
    });

    // CAP_BT_WHEN_POWERED_ON
    registerHandler(CAP_BT_WHEN_POWERED_ON, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setBluetoothWhenPoweredOn(h, getBool(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->enabled ? 1 : 0);
    });

    // CAP_BT_CALL_VOLUME
    registerHandler(CAP_BT_CALL_VOLUME, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setBluetoothCallVolume(h, getUint8(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->volume);
    });

    // CAP_NOISE_FILTER
    registerHandler(CAP_NOISE_FILTER, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setNoiseFilter(h, detail::getUint8(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->level);
    });

    // CAP_SIDETONE_STATUS
    registerHandler(CAP_SIDETONE_STATUS, [](HIDDevice* dev, hid_device* h, const FeatureParam&) -> Result<FeatureOutput> {
        auto r = dev->getSidetone(h);
        if (r.hasError())
            return r.error();
        return FeatureOutput::fromSidetone(r.value());
    });

    // CAP_ANC
    registerHandler(CAP_ANC, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setAnc(h, getUint8(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->mode);
    });

    // CAP_ANC_STARTUP_MODE
    registerHandler(CAP_ANC_STARTUP_MODE, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setAncStartupMode(h, getUint8(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->mode);
    });

    // CAP_ANC_BUTTON_MODES
    registerHandler(CAP_ANC_BUTTON_MODES, [](HIDDevice* dev, hid_device* h, const FeatureParam& p) -> Result<FeatureOutput> {
        auto r = dev->setAncButtonModes(h, getAncButtonModes(p));
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(0);
    });

    // CAP_MICROPHONE_ATTACHMENT_STATUS
    registerHandler(CAP_MICROPHONE_ATTACHMENT_STATUS, [](HIDDevice* dev, hid_device* h, const FeatureParam&) -> Result<FeatureOutput> {
        auto r = dev->getMicAttachmentStatus(h);
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->attached ? 1 : 0, r->attached ? "attached" : "detached");
    });

    // CAP_MICROPHONE_MUTE_STATUS
    registerHandler(CAP_MICROPHONE_MUTE_STATUS, [](HIDDevice* dev, hid_device* h, const FeatureParam&) -> Result<FeatureOutput> {
        auto r = dev->getMicMuteStatus(h);
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->muted ? 1 : 0, r->muted ? "muted" : "unmuted");
    });
}

inline void FeatureHandlerRegistry::registerAllReadHandlers()
{
    using namespace detail;

    registerReadHandler(CAP_SIDETONE, [](HIDDevice* dev, hid_device* h, const FeatureParam&) -> Result<FeatureOutput> {
        auto r = dev->getSidetone(h);
        if (r.hasError())
            return r.error();
        return FeatureOutput::fromSidetone(r.value());
    });

    registerReadHandler(CAP_ANC, [](HIDDevice* dev, hid_device* h, const FeatureParam&) -> Result<FeatureOutput> {
        auto r = dev->getAnc(h);
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->mode, std::string(ancModeName(r->mode)));
    });

    registerReadHandler(CAP_ANC_STARTUP_MODE, [](HIDDevice* dev, hid_device* h, const FeatureParam&) -> Result<FeatureOutput> {
        auto r = dev->getAncStartupMode(h);
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->mode, std::string(ancModeName(r->mode)));
    });

    // Value is a bitmask (off = 1, anc = 2, ambient = 4); the label uses the
    // comma-separated form --anc-button-modes takes.
    registerReadHandler(CAP_ANC_BUTTON_MODES, [](HIDDevice* dev, hid_device* h, const FeatureParam&) -> Result<FeatureOutput> {
        auto r = dev->getAncButtonModes(h);
        if (r.hasError())
            return r.error();
        std::string label;
        for (auto [enabled, name] : { std::pair { r->off, "off" }, std::pair { r->anc, "anc" }, std::pair { r->ambient, "ambient" } }) {
            if (enabled) {
                label += label.empty() ? name : std::format(",{}", name);
            }
        }
        const int mask = (r->off ? 1 : 0) | (r->anc ? 2 : 0) | (r->ambient ? 4 : 0);
        return FeatureOutput::success(mask, label);
    });

    registerReadHandler(CAP_INACTIVE_TIME, [](HIDDevice* dev, hid_device* h, const FeatureParam&) -> Result<FeatureOutput> {
        auto r = dev->getInactiveTime(h);
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->minutes);
    });

    registerReadHandler(CAP_VOICE_PROMPTS, [](HIDDevice* dev, hid_device* h, const FeatureParam&) -> Result<FeatureOutput> {
        auto r = dev->getVoicePrompts(h);
        if (r.hasError())
            return r.error();
        return FeatureOutput::success(r->enabled ? 1 : 0, r->enabled ? "on" : "off");
    });

    // 2 = restore the state at power off, which the 0/1 setter cannot select
    registerReadHandler(CAP_BT_WHEN_POWERED_ON, [](HIDDevice* dev, hid_device* h, const FeatureParam&) -> Result<FeatureOutput> {
        auto r = dev->getBluetoothWhenPoweredOn(h);
        if (r.hasError())
            return r.error();
        if (r->last_state)
            return FeatureOutput::success(2, "last");
        return FeatureOutput::success(r->enabled ? 1 : 0, r->enabled ? "on" : "off");
    });
}

} // namespace headsetcontrol
