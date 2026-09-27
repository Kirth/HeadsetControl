#pragma once

#include "../../result_types.hpp"
#include "../device_utils.hpp"
#include "../hid_device.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <vector>

namespace headsetcontrol::protocols {

/**
 * @brief Sony INZONE vendor HCI-over-HID protocol (INZONE H5, H9 II)
 *
 * The 2.4 GHz dongles expose the control protocol on a Sony-vendor HID
 * collection with usage page 0xFF04 (report ID 0x02, 63-byte payload). It is
 * a thin Sony vendor layer over standard Bluetooth HCI: each report carries
 * an HCI packet where the host issues commands with opcode 0xFC00 and the
 * dongle replies with vendor event code 0xFF.
 *
 * Which HID interface/collection carries the protocol differs per dongle, so
 * each device provides its own getCapabilityDetail().
 */
class SonyINZONEProtocol : public HIDDevice {
protected:
    static constexpr uint16_t VENDOR_SONY = 0x054C;

    // HID transport
    static constexpr int REPORT_SIZE   = 64;
    static constexpr uint8_t REPORT_ID = 0x02;

    // HCI shell constants
    static constexpr uint8_t HCI_TYPE_COMMAND = 0x01;
    static constexpr uint8_t HCI_TYPE_EVENT   = 0x04;
    static constexpr uint8_t SONY_EVENT_CODE  = 0xFF;
    static constexpr uint8_t SONY_OPCODE_LO   = 0x00; // 0xFC00 LE
    static constexpr uint8_t SONY_OPCODE_HI   = 0xFC;
    static constexpr uint8_t SONY_KEY_ID_LO   = 0x96;
    static constexpr uint8_t SONY_KEY_ID_HI   = 0xC3;

    // Command framing: report ID + 12 header bytes before the payload, and a
    // trailing checksum byte after it.
    static constexpr size_t MAX_PAYLOAD_SIZE = REPORT_SIZE - 14;

    // ADDRESS nibbles
    static constexpr uint8_t ADDR_PC       = 0x1;
    static constexpr uint8_t ADDR_TX       = 0x2;
    static constexpr uint8_t ADDR_RX       = 0x4;
    static constexpr uint8_t ADDR_PC_TO_RX = (ADDR_RX << 4) | ADDR_PC; // 0x41
    static constexpr uint8_t ADDR_PC_TO_TX = (ADDR_TX << 4) | ADDR_PC; // 0x21

    // EVENT_TYPE values
    static constexpr uint8_t ETYPE_GET         = 0x01;
    static constexpr uint8_t ETYPE_SET         = 0x02;
    static constexpr uint8_t ETYPE_RET         = 0x10;
    static constexpr uint8_t ETYPE_NTFY        = 0x20;
    static constexpr uint8_t ETYPE_NTFY_ACTIVE = 0xA0;

    // EVENT_ID values
    static constexpr uint8_t EID_2GHZ_CONNECT_STATUS    = 0x01;
    static constexpr uint8_t EID_BATTERY_INFO           = 0x04;
    static constexpr uint8_t EID_HEADPHONE_VOLUME       = 0x21;
    static constexpr uint8_t EID_GAME_CHAT_MIX_BALANCE  = 0x22;
    static constexpr uint8_t EID_SIDETONE_VOLUME        = 0x23;
    static constexpr uint8_t EID_MIC_VOLUME             = 0x24;
    static constexpr uint8_t EID_AMB_SETTING            = 0x41;
    static constexpr uint8_t EID_NC_TOGGLE_SETTING      = 0x42;
    static constexpr uint8_t EID_NC_STARTUP_MODE        = 0x43;
    static constexpr uint8_t EID_AUTO_POWER_OFF_SETTING = 0x81;
    static constexpr uint8_t EID_BT_STARTUP_MODE        = 0x63;
    static constexpr uint8_t EID_GUIDANCE_SETTING       = 0x84;
    static constexpr uint8_t EID_MIC_ATTACHED_STATUS    = 0x8F;

    // Device-side ranges. Headphone volume is 0..50 and balance is 0..90 in
    // steps of 10. Sidetone and mic ranges are not yet verified — assumed to
    // follow the headphone convention (0..50); the setters clamp via map().
    static constexpr uint8_t DEVICE_VOLUME_MAX   = 50;
    static constexpr uint8_t DEVICE_BALANCE_MAX  = 90; // step 10
    static constexpr uint8_t DEVICE_SIDETONE_MAX = 50;
    static constexpr uint8_t DEVICE_MIC_VOL_MAX  = 50;

    // AMB_SETTING nc_setting: 0 = off, 1 = NC, 2 = ambient
    static constexpr uint8_t NC_SETTING_MAX = 2;

    // NC_STARTUP_MODE: 0 = off, 1 = NC, 2 = ambient, 3 = mode at power off
    static constexpr uint8_t NC_STARTUP_MODE_MAX = 3;

    // BT_STARTUP_MODE: 0 = off, 1 = on, 2 = state at power off
    static constexpr uint8_t BT_STARTUP_OFF        = 0;
    static constexpr uint8_t BT_STARTUP_ON         = 1;
    static constexpr uint8_t BT_STARTUP_LAST_STATE = 2;

    // AUTO_POWER_OFF_SETTING minutes; 0 = never
    static constexpr std::array<uint8_t, 6> AUTO_POWER_OFF_MINUTES { 0, 5, 15, 30, 60, 180 };

    // Timeouts for matched-response wait
    static constexpr int READ_TIMEOUT_MS   = 500;
    static constexpr int MAX_READ_ATTEMPTS = 10;

    struct ParsedEvent {
        uint8_t event_id        = 0;
        uint8_t event_type      = 0;
        uint8_t address         = 0;
        uint16_t transaction_id = 0;
        std::vector<uint8_t> payload;
    };

public:
    // ------------------------------------------------------------------------
    // Payload decoders for GET replies. Static so they can be unit tested
    // without a device. 0xFF is the dongle's placeholder for "no cached value",
    // which it reports while the headset is offline.
    // ------------------------------------------------------------------------

    /// AMB_SETTING: [nc_setting, ambient_level, ambient_level_percent, voice_focus]
    /// Only nc_setting is decoded: it is the part CAP_ANC sets. The ambient
    /// level (1..20) and voice focus are configured in INZONE Hub.
    static Result<AncResult> parseAmbSetting(std::span<const uint8_t> payload)
    {
        if (payload.size() < 4) {
            return DeviceError::protocolError("AMB_SETTING payload too short");
        }
        auto mode = parseSingleByte(payload, NC_SETTING_MAX, "AMB_SETTING");
        if (!mode) {
            return mode.error();
        }
        return AncResult { .mode = *mode };
    }

    /// NC_TOGGLE_SETTING: [off_enable, nc_enable, ambient_enable]
    static Result<AncButtonModesResult> parseNcToggleSetting(std::span<const uint8_t> payload)
    {
        if (payload.size() < 3) {
            return DeviceError::protocolError("NC_TOGGLE_SETTING payload too short");
        }
        if (payload[0] == 0xFF) {
            return DeviceError::deviceOffline("Headset offline");
        }
        if (payload[0] > 1 || payload[1] > 1 || payload[2] > 1) {
            return DeviceError::protocolError(std::format(
                "Invalid ANC button modes: {} {} {}", payload[0], payload[1], payload[2]));
        }
        return AncButtonModesResult {
            .off     = payload[0] == 1,
            .anc     = payload[1] == 1,
            .ambient = payload[2] == 1,
        };
    }

    /// NC_STARTUP_MODE: [mode]
    static Result<AncStartupModeResult> parseNcStartupMode(std::span<const uint8_t> payload)
    {
        auto mode = parseSingleByte(payload, NC_STARTUP_MODE_MAX, "NC_STARTUP_MODE");
        if (!mode) {
            return mode.error();
        }
        return AncStartupModeResult { .mode = *mode };
    }

    /// BT_STARTUP_MODE: [mode]
    static Result<BluetoothWhenPoweredOnResult> parseBtStartupMode(std::span<const uint8_t> payload)
    {
        auto mode = parseSingleByte(payload, BT_STARTUP_LAST_STATE, "BT_STARTUP_MODE");
        if (!mode) {
            return mode.error();
        }
        return BluetoothWhenPoweredOnResult {
            .enabled    = *mode == BT_STARTUP_ON,
            .last_state = *mode == BT_STARTUP_LAST_STATE,
        };
    }

    /// AUTO_POWER_OFF_SETTING: [minutes] (H9 II adds a second byte, the last
    /// non-zero choice, which INZONE Hub uses to keep its dropdown selection
    /// while auto power off is disabled)
    static Result<InactiveTimeResult> parseAutoPowerOffSetting(std::span<const uint8_t> payload)
    {
        if (payload.empty()) {
            return DeviceError::protocolError("AUTO_POWER_OFF_SETTING payload empty");
        }
        const uint8_t minutes = payload[0];
        if (minutes == 0xFF) {
            return DeviceError::deviceOffline("Headset offline");
        }
        if (std::find(AUTO_POWER_OFF_MINUTES.begin(), AUTO_POWER_OFF_MINUTES.end(), minutes)
            == AUTO_POWER_OFF_MINUTES.end()) {
            return DeviceError::protocolError(std::format("Invalid auto power off time: {}", minutes));
        }
        return InactiveTimeResult {
            .minutes     = minutes,
            .min_minutes = 0,
            .max_minutes = AUTO_POWER_OFF_MINUTES.back(),
        };
    }

    /// GUIDANCE_SETTING: [enabled]
    static Result<VoicePromptsResult> parseGuidanceSetting(std::span<const uint8_t> payload)
    {
        auto enabled = parseSingleByte(payload, 1, "GUIDANCE_SETTING");
        if (!enabled) {
            return enabled.error();
        }
        return VoicePromptsResult { .enabled = *enabled == 1 };
    }

    /// SIDETONE_VOLUME: [value, percent]
    static Result<SidetoneResult> parseSidetoneVolume(std::span<const uint8_t> payload)
    {
        auto value = parseSingleByte(payload, DEVICE_SIDETONE_MAX, "SIDETONE_VOLUME");
        if (!value) {
            return value.error();
        }
        return SidetoneResult {
            .current_level = map<uint8_t>(*value, 0, DEVICE_SIDETONE_MAX, 0, 128),
            .min_level     = 0,
            .max_level     = 128,
            .device_min    = 0,
            .device_max    = DEVICE_SIDETONE_MAX,
            .device_level  = *value,
        };
    }

protected:
    constexpr uint16_t getVendorId() const override { return VENDOR_SONY; }

    Result<BatteryResult> getSonyBattery(hid_device* device_handle)
    {
        auto resp = exchange(device_handle, ADDR_PC_TO_RX, EID_BATTERY_INFO, ETYPE_GET, {});
        if (!resp) {
            return resp.error();
        }
        const auto& payload = resp->payload;
        if (payload.size() < 2) {
            return DeviceError::protocolError("BATTERY_INFO payload too short");
        }

        const uint8_t charger = payload[0];
        const uint8_t percent = payload[1];

        // 0xFF placeholder = headset offline / no cached value
        if (percent == 0xFF) {
            return DeviceError::deviceOffline("Headset reports battery=0xFF (offline)");
        }
        if (percent > 100) {
            return DeviceError::protocolError(
                std::format("Invalid battery percent: {}", percent));
        }

        return BatteryResult {
            .level_percent = percent,
            .status        = (charger != 0) ? BATTERY_CHARGING : BATTERY_AVAILABLE,
            .raw_data      = payload,
        };
    }

    Result<ChatmixResult> getSonyChatmix(hid_device* device_handle)
    {
        auto resp = exchange(device_handle, ADDR_PC_TO_RX, EID_GAME_CHAT_MIX_BALANCE, ETYPE_GET, {});
        if (!resp) {
            return resp.error();
        }
        const auto& payload = resp->payload;
        if (payload.empty()) {
            return DeviceError::protocolError("GAME_CHAT_MIX_BALANCE payload empty");
        }

        // payload[0] = mixBalance: 0..90 in steps of 10, 0=full game, 90=full chat.
        const uint8_t balance = payload[0];
        if (balance == 0xFF) {
            return DeviceError::deviceOffline("Headset offline");
        }
        if (balance > DEVICE_BALANCE_MAX) {
            return DeviceError::protocolError(
                std::format("Invalid balance: {}", balance));
        }

        const int chat_pct = (balance * 100) / DEVICE_BALANCE_MAX;
        const int game_pct = 100 - chat_pct;
        const int level    = map<int>(balance, 0, DEVICE_BALANCE_MAX, 0, 128);

        return ChatmixResult {
            .level               = level,
            .game_volume_percent = game_pct,
            .chat_volume_percent = chat_pct,
        };
    }

    Result<SidetoneResult> setSonySidetone(hid_device* device_handle, uint8_t level)
    {
        const uint8_t dev_level = map<uint8_t>(level, 0, 128, 0, DEVICE_SIDETONE_MAX);
        // SIDETONE_VOLUME payload: [sidetoneVolValue, sidetoneVolPercent]
        // The percent byte is a UI label the Hub never reads; it echoes back
        // the last value it received, or 0xFF before it has received one.
        const std::array<uint8_t, 2> payload { dev_level, 0xFF };

        auto resp = exchange(device_handle, ADDR_PC_TO_RX, EID_SIDETONE_VOLUME, ETYPE_SET,
            std::span<const uint8_t> { payload });
        if (!resp) {
            return resp.error();
        }

        return SidetoneResult {
            .current_level = level,
            .min_level     = 0,
            .max_level     = 128,
            .device_min    = 0,
            .device_max    = DEVICE_SIDETONE_MAX,
        };
    }

    Result<MicVolumeResult> setSonyMicVolume(hid_device* device_handle, uint8_t volume)
    {
        const uint8_t dev_level = map<uint8_t>(volume, 0, 128, 0, DEVICE_MIC_VOL_MAX);
        // MIC_VOLUME payload: [micMute, micVolValue, micVolPercent]
        const std::array<uint8_t, 3> payload { 0x00, dev_level, 0xFF };

        auto resp = exchange(device_handle, ADDR_PC_TO_RX, EID_MIC_VOLUME, ETYPE_SET,
            std::span<const uint8_t> { payload });
        if (!resp) {
            return resp.error();
        }

        return MicVolumeResult {
            .volume     = volume,
            .min_volume = 0,
            .max_volume = 128,
        };
    }

    Result<InactiveTimeResult> setSonyInactiveTime(hid_device* device_handle, uint8_t minutes, bool h9ii_format)
    {
        if (minutes != 0 && minutes != 5 && minutes != 15 && minutes != 30 && minutes != 60 && minutes != 180) {
            return DeviceError::invalidParameter("Sony INZONE auto power off supports 0, 5, 15, 30, 60, or 180 minutes");
        }

        const std::array<uint8_t, 2> payload { minutes, minutes };
        auto resp = exchange(device_handle, ADDR_PC_TO_RX, EID_AUTO_POWER_OFF_SETTING, ETYPE_SET,
            std::span<const uint8_t> { payload }.first(h9ii_format ? 2 : 1));
        if (!resp) {
            return resp.error();
        }

        return InactiveTimeResult {
            .minutes     = minutes,
            .min_minutes = 0,
            .max_minutes = 180,
        };
    }

    Result<VoicePromptsResult> setSonyVoicePrompts(hid_device* device_handle, bool enabled)
    {
        const std::array<uint8_t, 1> payload { static_cast<uint8_t>(enabled ? 1 : 0) };
        auto resp = exchange(device_handle, ADDR_PC_TO_RX, EID_GUIDANCE_SETTING, ETYPE_SET,
            std::span<const uint8_t> { payload });
        if (!resp) {
            return resp.error();
        }

        return VoicePromptsResult { .enabled = enabled };
    }

    Result<MicAttachmentStatusResult> getSonyMicAttachmentStatus(hid_device* device_handle)
    {
        auto resp = exchange(device_handle, ADDR_PC_TO_RX, EID_MIC_ATTACHED_STATUS, ETYPE_GET, {});
        if (!resp) {
            return resp.error();
        }
        if (resp->payload.empty()) {
            return DeviceError::protocolError("MIC_ATTACHED_STATUS payload empty");
        }

        // payload[0] is a "removed" flag despite the event name. Observed on an
        // H9 II: 0x01 with the boom mic unplugged, 0x00 with it plugged in.
        const uint8_t removed = resp->payload[0];
        if (removed == 0xFF) {
            return DeviceError::deviceOffline("Headset offline");
        }
        if (removed > 1) {
            return DeviceError::protocolError(
                std::format("Invalid mic attachment status: {}", removed));
        }
        return MicAttachmentStatusResult { .attached = (removed == 0) };
    }

    Result<MicMuteStatusResult> getSonyMicMuteStatus(hid_device* device_handle)
    {
        auto resp = exchange(device_handle, ADDR_PC_TO_RX, EID_MIC_VOLUME, ETYPE_GET, {});
        if (!resp) {
            return resp.error();
        }
        if (resp->payload.empty()) {
            return DeviceError::protocolError("MIC_VOLUME payload empty");
        }
        if (resp->payload[0] != 0 && resp->payload[0] != 1) {
            return DeviceError::protocolError(
                std::format("Invalid mic mute status: {}", resp->payload[0]));
        }
        return MicMuteStatusResult { .muted = (resp->payload[0] == 1) };
    }

    Result<AncStartupModeResult> setSonyAncStartupMode(hid_device* device_handle, uint8_t mode)
    {
        if (mode > 3) {
            return DeviceError::invalidParameter("ANC startup mode must be 0 (off), 1 (NC), 2 (ambient), or 3 (mode at power off)");
        }
        const std::array<uint8_t, 1> payload { mode };
        auto resp = exchange(device_handle, ADDR_PC_TO_RX, EID_NC_STARTUP_MODE, ETYPE_SET,
            std::span<const uint8_t> { payload });
        if (!resp) {
            return resp.error();
        }
        return AncStartupModeResult { .mode = mode };
    }

    Result<AncResult> setSonyAnc(hid_device* device_handle, uint8_t mode)
    {
        if (mode > 2) {
            return DeviceError::invalidParameter("ANC mode must be 0 (off), 1 (ANC), or 2 (ambient sound)");
        }

        // AMB_SETTING payload: [nc_mode, ambient_level, ambient_level_percent, voice_focus]
        // The SET always carries all four bytes, so read the current setting
        // first and only replace nc_mode. That keeps the ambient level and
        // voice focus configured in INZONE Hub.
        // Observed GET reply on an H9 II: 01 14 FF 00 (ANC, level 20, 0xFF, off).
        // The headset applies a new mode asynchronously: a GET sent right after
        // the SET can still report the previous nc_mode for a moment.
        auto current = exchange(device_handle, ADDR_PC_TO_RX, EID_AMB_SETTING, ETYPE_GET, {});
        if (!current) {
            return current.error();
        }
        if (current->payload.size() < 4) {
            return DeviceError::protocolError("AMB_SETTING payload too short");
        }

        std::array<uint8_t, 4> payload {};
        std::copy_n(current->payload.begin(), payload.size(), payload.begin());
        payload[0] = mode;

        auto resp = exchange(device_handle, ADDR_PC_TO_RX, EID_AMB_SETTING, ETYPE_SET,
            std::span<const uint8_t> { payload });
        if (!resp) {
            return resp.error();
        }
        return AncResult { .mode = mode };
    }

    Result<AncButtonModesResult> setSonyAncButtonModes(
        hid_device* device_handle, const AncButtonModes& modes)
    {
        if (!modes.any()) {
            return DeviceError::invalidParameter("At least one ANC button mode must be enabled");
        }

        const std::array<uint8_t, 3> payload {
            static_cast<uint8_t>(modes.off ? 1 : 0),
            static_cast<uint8_t>(modes.anc ? 1 : 0),
            static_cast<uint8_t>(modes.ambient ? 1 : 0),
        };
        auto resp = exchange(device_handle, ADDR_PC_TO_RX, EID_NC_TOGGLE_SETTING, ETYPE_SET,
            std::span<const uint8_t> { payload });
        if (!resp) {
            return resp.error();
        }
        return AncButtonModesResult {
            .off     = modes.off,
            .anc     = modes.anc,
            .ambient = modes.ambient,
        };
    }

    Result<BluetoothWhenPoweredOnResult> setSonyBluetoothWhenPoweredOn(hid_device* device_handle, bool enabled)
    {
        const std::array<uint8_t, 1> payload { static_cast<uint8_t>(enabled ? 1 : 0) };
        auto resp = exchange(device_handle, ADDR_PC_TO_RX, EID_BT_STARTUP_MODE, ETYPE_SET,
            std::span<const uint8_t> { payload });
        if (!resp) {
            return resp.error();
        }

        return BluetoothWhenPoweredOnResult { .enabled = enabled };
    }

    Result<SidetoneResult> getSonySidetone(hid_device* device_handle)
    {
        return getAndParse(device_handle, EID_SIDETONE_VOLUME, parseSidetoneVolume);
    }

    Result<AncResult> getSonyAnc(hid_device* device_handle)
    {
        return getAndParse(device_handle, EID_AMB_SETTING, parseAmbSetting);
    }

    Result<AncStartupModeResult> getSonyAncStartupMode(hid_device* device_handle)
    {
        return getAndParse(device_handle, EID_NC_STARTUP_MODE, parseNcStartupMode);
    }

    Result<AncButtonModesResult> getSonyAncButtonModes(hid_device* device_handle)
    {
        return getAndParse(device_handle, EID_NC_TOGGLE_SETTING, parseNcToggleSetting);
    }

    Result<InactiveTimeResult> getSonyInactiveTime(hid_device* device_handle)
    {
        return getAndParse(device_handle, EID_AUTO_POWER_OFF_SETTING, parseAutoPowerOffSetting);
    }

    Result<VoicePromptsResult> getSonyVoicePrompts(hid_device* device_handle)
    {
        return getAndParse(device_handle, EID_GUIDANCE_SETTING, parseGuidanceSetting);
    }

    Result<BluetoothWhenPoweredOnResult> getSonyBluetoothWhenPoweredOn(hid_device* device_handle)
    {
        return getAndParse(device_handle, EID_BT_STARTUP_MODE, parseBtStartupMode);
    }

    /**
     * @brief GET an RX setting and decode the RET payload.
     *
     * INZONE Hub itself reads settings from the ALL_FUNCTION_SETTINGS_PART1..3
     * aggregates at enumeration, but the dongle answers per-event GETs too.
     */
    template <typename Parser>
    auto getAndParse(hid_device* device_handle, uint8_t event_id, Parser parser)
        -> decltype(parser(std::span<const uint8_t> {}))
    {
        auto resp = exchange(device_handle, ADDR_PC_TO_RX, event_id, ETYPE_GET, {});
        if (!resp) {
            return resp.error();
        }
        return parser(std::span<const uint8_t> { resp->payload });
    }

    /// Decode a one-byte setting in 0..max, rejecting the 0xFF offline placeholder.
    static Result<uint8_t> parseSingleByte(std::span<const uint8_t> payload, uint8_t max, std::string_view name)
    {
        if (payload.empty()) {
            return DeviceError::protocolError(std::format("{} payload empty", name));
        }
        if (payload[0] == 0xFF) {
            return DeviceError::deviceOffline("Headset offline");
        }
        if (payload[0] > max) {
            return DeviceError::protocolError(std::format("Invalid {} value: {}", name, payload[0]));
        }
        return payload[0];
    }

    /**
     * @brief Send an HCI COMMAND and wait for the matching EVENT response.
     *
     * The dongle responds to a GET with EVENT_TYPE.RET and to a SET with
     * EVENT_TYPE.NTFY, both carrying back the same TID we sent. Unsolicited
     * NTFY_ACTIVE events may arrive on the same channel; we skip frames that
     * don't match our request.
     */
    Result<ParsedEvent> exchange(hid_device* device_handle, uint8_t address,
        uint8_t event_id, uint8_t event_type, std::span<const uint8_t> payload)
    {
        if (payload.size() > MAX_PAYLOAD_SIZE) {
            return DeviceError::invalidParameter(
                std::format("Payload of {} bytes exceeds maximum of {}", payload.size(), MAX_PAYLOAD_SIZE));
        }

        uint16_t tid = ++transaction_counter_;
        if (tid <= 1) {
            // Skip TID 0 (overflow) and TID 1: the dongle's own unsolicited
            // NTFY_ACTIVE pushes carry TID=1, so reusing it would let a push be
            // matched as our reply.
            tid = transaction_counter_ = 2;
        }

        std::array<uint8_t, REPORT_SIZE> buf {};
        buildCommand(buf, address, event_id, event_type, tid, payload);

        if (auto wr = writeHID(device_handle, buf); !wr) {
            return wr.error();
        }

        const uint8_t want_type = (event_type == ETYPE_SET) ? ETYPE_NTFY : ETYPE_RET;

        for (int attempt = 0; attempt < MAX_READ_ATTEMPTS; ++attempt) {
            std::array<uint8_t, REPORT_SIZE> resp {};
            auto rd = readHIDTimeout(device_handle, resp, READ_TIMEOUT_MS);
            if (!rd) {
                if (rd.error().code == DeviceError::Code::Timeout) {
                    continue;
                }
                return rd.error();
            }

            auto parsed = parseEvent(resp);
            if (!parsed) {
                continue; // not an HCI event, or failed validation
            }

            // Prefer a response that matches both event_id and our TID
            // (which also implicitly filters out unsolicited NTFY_ACTIVE,
            // since those carry TID=1 originated by the dongle).
            if (parsed->event_id == event_id
                && parsed->transaction_id == tid
                && (parsed->event_type == want_type
                    || parsed->event_type == ETYPE_NTFY_ACTIVE)) {
                return *parsed;
            }
            // Otherwise keep reading — it may be an unrelated NTFY_ACTIVE.
        }

        return DeviceError::timeout(
            std::format("No response for event_id 0x{:02x} (TID {})", event_id, tid));
    }

    /**
     * @brief Build a Sony vendor HCI COMMAND in the host write buffer.
     *
     * Layout (post-report-ID offsets):
     *   [0]      hid_length  = 12 + len(payload)
     *   [1]      hci_type    = 0x01 (COMMAND)
     *   [2..3]   opcode      = 0xFC00 (LE)
     *   [4]      param_length = 8 + len(payload)
     *   [5..6]   sony_key_id = 0xC396 (LE)
     *   [7]      address     = (Dst<<4) | Src
     *   [8]      event_id
     *   [9]      event_type
     *   [10..11] transaction_id (LE)
     *   [12..]   payload
     *   [..]     checksum    = sum(post-rid[4..end-1]) & 0xFF
     *
     * Map to buf indices (buf[0] = report ID): each post-rid offset N maps to
     * buf[N+1].
     *
     * The caller must ensure payload.size() <= MAX_PAYLOAD_SIZE.
     */
    static void buildCommand(std::array<uint8_t, REPORT_SIZE>& buf,
        uint8_t address, uint8_t event_id, uint8_t event_type,
        uint16_t tid, std::span<const uint8_t> payload)
    {
        const size_t payload_len = std::min(payload.size(), MAX_PAYLOAD_SIZE);
        const size_t hid_length  = 12 + payload_len; // HCI byte count

        buf[0] = REPORT_ID;
        buf[1] = static_cast<uint8_t>(hid_length);

        buf[2]  = HCI_TYPE_COMMAND;
        buf[3]  = SONY_OPCODE_LO;
        buf[4]  = SONY_OPCODE_HI;
        buf[5]  = static_cast<uint8_t>(8 + payload_len); // param_length
        buf[6]  = SONY_KEY_ID_LO;
        buf[7]  = SONY_KEY_ID_HI;
        buf[8]  = address;
        buf[9]  = event_id;
        buf[10] = event_type;
        buf[11] = static_cast<uint8_t>(tid & 0xFF);
        buf[12] = static_cast<uint8_t>((tid >> 8) & 0xFF);

        for (size_t i = 0; i < payload_len; ++i) {
            buf[13 + i] = payload[i];
        }

        // checksum = sum(buf[6..12+payload_len]) & 0xFF
        unsigned sum = 0;
        for (size_t i = 6; i <= 12 + payload_len; ++i) {
            sum += buf[i];
        }
        buf[13 + payload_len] = static_cast<uint8_t>(sum & 0xFF);
    }

    /**
     * @brief Validate and parse an incoming HCI EVENT report.
     *
     * Returns std::nullopt for any frame that isn't a well-formed Sony
     * vendor EVENT addressed to the host. The HCI checksum is verified.
     */
    static std::optional<ParsedEvent> parseEvent(const std::array<uint8_t, REPORT_SIZE>& buf)
    {
        if (buf[0] != REPORT_ID) {
            return std::nullopt;
        }
        const uint8_t hid_length = buf[1];
        if (hid_length < 12 || hid_length > REPORT_SIZE - 2) {
            return std::nullopt;
        }

        // HCI shell checks
        if (buf[2] != HCI_TYPE_EVENT)
            return std::nullopt;
        if (buf[3] != SONY_EVENT_CODE)
            return std::nullopt;
        // buf[4] = param_length (informational; ignore)
        if (buf[5] != 0x00) // dummy byte
            return std::nullopt;
        if (buf[6] != SONY_KEY_ID_LO || buf[7] != SONY_KEY_ID_HI)
            return std::nullopt;

        const uint8_t address = buf[8];
        // Destination must be PC (high nibble == 1)
        if ((address >> 4) != ADDR_PC)
            return std::nullopt;

        // Checksum: sum(HCI[3..N-1]) & 0xFF == HCI[N], where HCI[i] = buf[i+2].
        // i.e. sum(buf[5..hid_length]) & 0xFF == buf[hid_length+1].
        unsigned sum = 0;
        for (size_t i = 5; i <= hid_length; ++i) {
            sum += buf[i];
        }
        if (static_cast<uint8_t>(sum & 0xFF) != buf[hid_length + 1]) {
            return std::nullopt;
        }

        // Payload runs from buf[13] up to buf[hid_length] inclusive.
        return ParsedEvent {
            .event_id       = buf[9],
            .event_type     = buf[10],
            .address        = address,
            .transaction_id = static_cast<uint16_t>(buf[11] | (buf[12] << 8)),
            .payload        = (hid_length > 12)
                       ? std::vector<uint8_t>(buf.begin() + 13, buf.begin() + hid_length + 1)
                       : std::vector<uint8_t> {},
        };
    }

private:
    uint16_t transaction_counter_ = 0;
};

} // namespace headsetcontrol::protocols
