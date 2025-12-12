#include <cstdlib>
#include <cstring>
#include <esp_err.h>
#include <esp_log.h>
#include <font_awesome.h>
#include <string>

#include "application.h"
#include "assets/lang_config.h"
#include "audio_codec.h"
#include "board.h"
#include "display.h"
#include "servo_control_board.h"
#include "settings.h"

#define TAG "Display"

Display::Display() {}

Display::~Display() {}

void Display::SetStatus(const char* status) { ESP_LOGW(TAG, "SetStatus: %s", status); }

void Display::ShowNotification(const std::string& notification, int duration_ms) { ShowNotification(notification.c_str(), duration_ms); }

void Display::ShowNotification(const char* notification, int duration_ms) { ESP_LOGW(TAG, "ShowNotification: %s", notification); }

void Display::UpdateStatusBar(bool update_all) {}

struct Emotion {
    const char* text;
    std::function<void()> activate_func;
};
void Display::SetEmotion(const char* emotion) {
    ESP_LOGW(TAG, "SetEmotion: %s", emotion);

    static const std::vector<Emotion> emotions = {{"neutral", [this]() { scb.neutral(); }},
                                                  {"happy", [this]() { scb.speaking_state(); }},
                                                  {"laughing", [this]() { scb.speaking_state(); }},
                                                  {"funny", [this]() { scb.speaking_state(); }},
                                                  {"sad", [this]() {}},
                                                  {"angry", [this]() {}},
                                                  {"crying", [this]() {}},
                                                  {"loving", [this]() { scb.speaking_state(); }},
                                                  {"embarrassed", [this]() { scb.speaking_state(); }},
                                                  {"surprised", [this]() { scb.speaking_state(); }},
                                                  {"shocked", [this]() { scb.speaking_state(); }},
                                                  {"thinking", [this]() { scb.speaking_state(); }},
                                                  {"winking", [this]() { scb.speaking_state(); }},
                                                  {"cool", [this]() { scb.speaking_state(); }},
                                                  {"relaxed", [this]() { scb.speaking_state(); }},
                                                  {"delicious", [this]() { scb.speaking_state(); }},
                                                  {"kissy", [this]() { scb.speaking_state(); }},
                                                  {"confident", [this]() { scb.speaking_state(); }},
                                                  {"sleepy", [this]() { scb.sleepy(); }},
                                                  {"silly", [this]() { scb.speaking_state(); }},
                                                  {"confused", [this]() { scb.speaking_state(); }}};

    // 查找匹配的表情
    std::string_view emotion_view(emotion);
    auto it = std::find_if(emotions.begin(), emotions.end(), [&emotion_view](const Emotion& e) { return e.text == emotion_view; });
    DisplayLockGuard lock(this);

    // 如果找到匹配的表情就显示对应图标,否则显示默认的neutral表情
    if (it != emotions.end()) {
        ESP_LOGI(TAG, "已找到对应表情图标: %s", it->text);
        it->activate_func();
    } else {
        ESP_LOGW(TAG, "未找到对应表情图标");
        scb.neutral();
    }
}

void Display::SetChatMessage(const char* role, const char* content) {
    ESP_LOGW(TAG, "Role:%s", role);
    ESP_LOGW(TAG, "     %s", content);
}

void Display::SetTheme(Theme* theme) {
    current_theme_ = theme;
    Settings settings("display", true);
    settings.SetString("theme", theme->name());
}

void Display::SetPowerSaveMode(bool on) { ESP_LOGW(TAG, "SetPowerSaveMode: %d", on); }
