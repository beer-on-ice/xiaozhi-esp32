#include <cJSON.h>
#include <driver/uart.h>
#include <esp_log.h>

#include "application.h"
#include "board.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "mcp_server.h"
#include "mqtt_protocol.h"
#include "servo_control_board.h"

static const char* TAG = "ServoControlBoard";

ServoControlBoard::ServoControlBoard(void) {
    init_uart1();
    add_mcp_tools();
}

void ServoControlBoard::add_mcp_tools() {
    auto& mcp_server = McpServer::GetInstance();
    mcp_server.AddTool("self.system_action.reboot", "系统_动作->重启设备", PropertyList(),
                       [this](const PropertyList& properties) -> ReturnValue {
                           esp_restart();
                           return true;
                       });
    mcp_server.AddTool("self.system_action.instructionMode", "系统_动作->指令模式", PropertyList(),
                       [this](const PropertyList& properties) -> ReturnValue {
                           instruction_mode_ = true;
                           return true;
                       });
    mcp_server.AddTool("self.system_action.ChatMode", "系统_动作->聊天模式", PropertyList(),
                       [this](const PropertyList& properties) -> ReturnValue {
                           instruction_mode_ = false;
                           return true;
                       });
    mcp_server.AddTool("self.head_action.left", "脸部_动作->向左看", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
        uart_write_bytes(UART_NUM_1, DM12_Speed_36_Position_120, sizeof(DM12_Speed_36_Position_120));
        return true;
    });
    mcp_server.AddTool("self.head_action.right", "脸部_动作->向右看", PropertyList(),
                       [this](const PropertyList& properties) -> ReturnValue {
                           uart_write_bytes(UART_NUM_1, DM12_Speed_36_Position_60, sizeof(DM12_Speed_36_Position_60));
                           return true;
                       });
    mcp_server.AddTool("self.head_action.front", "脸部_动作->向前看", PropertyList(),
                       [this](const PropertyList& properties) -> ReturnValue {
                           uart_write_bytes(UART_NUM_1, DM12_Speed_18_Position_90, sizeof(DM12_Speed_18_Position_90));
                           return true;
                       });
    mcp_server.AddTool("self.face_action.smiled", "脸部_动作->微笑", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
        uart_write_bytes(UART_NUM_1, DM1_Speed_90_Position_60, sizeof(DM1_Speed_90_Position_60));
        uart_write_bytes(UART_NUM_1, DM2_Speed_90_Position_120, sizeof(DM2_Speed_90_Position_120));
        vTaskDelay(pdMS_TO_TICKS(2 * 1000));
        return true;
    });
    mcp_server.AddTool("self.face_action.grimace", "脸部_动作->做鬼脸", PropertyList(),
                       [this](const PropertyList& properties) -> ReturnValue {
                           vTaskSuspend(faceAction_eyelid_handle_);
                           uart_write_bytes(UART_NUM_1, DM5_Speed_90_Position_115, sizeof(DM5_Speed_90_Position_115));
                           uart_write_bytes(UART_NUM_1, DM6_Speed_90_Position_115, sizeof(DM6_Speed_90_Position_115));
                           uart_write_bytes(UART_NUM_1, DM0_Speed_18_Position_70, sizeof(DM0_Speed_18_Position_70));
                           uart_write_bytes(UART_NUM_1, DM1_Speed_90_Position_60, sizeof(DM1_Speed_90_Position_60));
                           uart_write_bytes(UART_NUM_1, DM2_Speed_90_Position_60, sizeof(DM2_Speed_90_Position_60));
                           vTaskDelay(pdMS_TO_TICKS(1000));
                           uart_write_bytes(UART_NUM_1, DM5_Speed_90_Position_65, sizeof(DM5_Speed_90_Position_65));
                           uart_write_bytes(UART_NUM_1, DM6_Speed_90_Position_65, sizeof(DM6_Speed_90_Position_65));
                           uart_write_bytes(UART_NUM_1, DM0_Speed_18_Position_70, sizeof(DM0_Speed_18_Position_70));
                           uart_write_bytes(UART_NUM_1, DM1_Speed_90_Position_120, sizeof(DM1_Speed_90_Position_120));
                           uart_write_bytes(UART_NUM_1, DM2_Speed_90_Position_120, sizeof(DM2_Speed_90_Position_120));
                           vTaskDelay(pdMS_TO_TICKS(2 * 1000));
                           vTaskResume(faceAction_eyelid_handle_);
                           return true;
                       });
    mcp_server.AddTool("self.face_action.wink", "脸部_动作->放个电", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
        vTaskSuspend(faceAction_eyelid_handle_);
        uart_write_bytes(UART_NUM_1, DM6_Speed_90_Position_115, sizeof(DM6_Speed_90_Position_115));
        uart_write_bytes(UART_NUM_1, DM2_Speed_90_Position_120, sizeof(DM2_Speed_90_Position_120));
        vTaskDelay(pdMS_TO_TICKS(2 * 1000));
        vTaskResume(faceAction_eyelid_handle_);
        return true;
    });
}

void ServoControlBoard::udp_send(const std::string& host, int port, const std::string& text) {
    if (!udp_instance_) {
        auto network = Board::GetInstance().GetNetwork();
        udp_instance_ = network->CreateUdp(3);
        udp_instance_->Connect(ESP32C6_IP, ESP32C6_PORT);
    }
    udp_instance_->Send(text);
}

void ServoControlBoard::init_uart1() {
    uart_config_t uart_config = {
        .baud_rate = UART1_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART1_TX_PIN, UART1_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGW(TAG, ">>> UART1 初始化完成 (TX: GPIO%d, RX: GPIO%d, baud_rate: %d)", UART1_TX_PIN, UART1_RX_PIN, uart_config.baud_rate);
}

void ServoControlBoard::init_esp_now() {

    // 1. 初始化 TCP/IP 网络(虽然不用 Wi-Fi AP,但必须初始化)
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 2. 设置 Wi-Fi 为 Station 模式
    // wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    // ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    // ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    // ESP_ERROR_CHECK(esp_wifi_start());

    // 3.获取当前AP信息
    esp_err_t ret;
    wifi_ap_record_t ap_info;
    ret = esp_wifi_sta_get_ap_info(&ap_info);
    if (ret == ESP_OK) {
        ESP_LOGW(TAG, "已连接到 AP:");
        ESP_LOGW(TAG, "  SSID: %s", ap_info.ssid);
        ESP_LOGW(TAG, "  BSSID: %02x:%02x:%02x:%02x:%02x:%02x", ap_info.bssid[0], ap_info.bssid[1], ap_info.bssid[2], ap_info.bssid[3],
                 ap_info.bssid[4], ap_info.bssid[5]);
        ESP_LOGW(TAG, "  信号强度: %d dBm", ap_info.rssi);
        ESP_LOGW(TAG, "  工作信道: %d", ap_info.primary);
        ESP_LOGW(TAG, "  带宽: %s",
                 ap_info.second == WIFI_SECOND_CHAN_NONE
                     ? "20MHz"
                     : (ap_info.second == WIFI_SECOND_CHAN_ABOVE ? "20/40MHz (Above)" : "20/40MHz (Below)"));
    } else if (ret == ESP_ERR_WIFI_NOT_CONNECT) {
        ESP_LOGW(TAG, "Wi-Fi 尚未连接");
    } else {
        ESP_LOGE(TAG, "获取 AP 信息失败: %s", esp_err_to_name(ret));
    }
    // 4.初始化 ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    // 5.注册接收回调
    ret = esp_now_register_recv_cb([](const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int data_len) {
        if (data_len > 0) {
            /**
             * {
             *   "monitor_id" : "98:A3:16:61:BA:7C", "data_group" : 3148, "data_type" : "HumanDectectedMonitor", "data" : {
             *      "x_point" : 0.12687, "y_point" : 0.555683, "move_speed" : 0
             *    }
             * }
             */
            // ESP_LOGI(TAG, ">>> Received from mac: %02x:%02x:%02x:%02x:%02x:%02x", esp_now_info->src_addr[0], esp_now_info->src_addr[1],
            //          esp_now_info->src_addr[2], esp_now_info->src_addr[3], esp_now_info->src_addr[4], esp_now_info->src_addr[5]);
            // ESP_LOGI(TAG, ">>> Msg: %s", data);

            const char* str = reinterpret_cast<const char*>(data);
            cJSON* object = cJSON_Parse(str);
            cJSON* data_type = cJSON_GetObjectItemCaseSensitive(object, "data_type");
            if (strcmp(data_type->valuestring, "HumanDectectedMonitor") == 0) {
                ESP_LOGI(TAG, ">>> Msg: %s", data);
                cJSON* data_json = cJSON_GetObjectItemCaseSensitive(object, "data");
                if (cJSON_IsObject(data_json)) {
                    cJSON* x_point = cJSON_GetObjectItemCaseSensitive(data_json, "x_point");
                    // ESP_LOGW(TAG, ">>> x_point: %f", x_point->valuedouble);
                    if (x_point->valuedouble < -0.22) {
                        uart_write_bytes(UART_NUM_1, DM12_Speed_36_Position_60, sizeof(DM12_Speed_36_Position_60));
                    } else if (x_point->valuedouble > 0.22) {
                        uart_write_bytes(UART_NUM_1, DM12_Speed_36_Position_120, sizeof(DM12_Speed_36_Position_120));
                    } else {
                        uart_write_bytes(UART_NUM_1, DM12_Speed_18_Position_90, sizeof(DM12_Speed_18_Position_90));
                    }
                }
            }
            cJSON_Delete(object);
        } else {
            ESP_LOGW(TAG, ">>> Received invalid data length: %d", data_len);
        }
        // 5.判断是否为广播包
        // bool is_broadcast = (memcmp(esp_now_info->des_addr, "\xFF\xFF\xFF\xFF\xFF\xFF", 6) == 0);
        // if (is_broadcast) {
        //     ESP_LOGI(TAG, ">>> This is a BROADCAST packet!");
        // }
    });
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP NOW 注册接收回调函数失败,错误码: 0x%X", ret);
    }
    // 6.获取并打印本机 MAC 地址(用于发送端配置)
    uint8_t mac_addr[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac_addr);
    ESP_LOGW(TAG, "==>> My MAC Address: %02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
             mac_addr[5]);
}

void ServoControlBoard::init_mqtt() {

    uint8_t mac[6];
    esp_err_t err = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (err == ESP_OK) {
        // char buf[18]; // 格式 "xx:xx:xx:xx:xx:xx" 共 17 字符 + 1 结尾 '\0'
        // snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        // mac_ = std::string(buf);

        char buf[44];
        snprintf(buf, sizeof(buf), mqtt_publish_topic_log_deviceState_.c_str(), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        mqtt_publish_topic_log_deviceState_ = std::string(buf);
        snprintf(buf, sizeof(buf), mqtt_publish_topic_mcp_mobileChassis_.c_str(), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        mqtt_publish_topic_mcp_mobileChassis_ = std::string(buf);

        ESP_LOGW(TAG, "==>> WiFi STA MAC: %s", mac_.c_str());
        ESP_LOGW(TAG, "==>> MQTT PUBLICSH [%s, %s]", mqtt_publish_topic_log_deviceState_.c_str(),
                 mqtt_publish_topic_mcp_mobileChassis_.c_str());
    } else {
        ESP_LOGE(TAG, "Failed to read MAC address");
    }
    err = esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    if (err == ESP_OK) {
        ESP_LOGW(TAG, "==>> WiFi AP MAC: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    auto network = Board::GetInstance().GetNetwork();
    mqtt_instance_ = network->CreateMqtt(1);
    mqtt_instance_->SetKeepAlive(240);

    mqtt_instance_->OnDisconnected([this]() { ESP_LOGW(TAG, "==>> MQTT disconnected!"); });

    mqtt_instance_->OnConnected([this]() { ESP_LOGW(TAG, "==>> MQTT connected!"); });

    mqtt_instance_->OnMessage([this](const std::string& topic, const std::string& payload) {
        ESP_LOGI(TAG, "==>> Topic: %s,Payload: %s", topic.c_str(), payload.c_str());
        if (topic == MQTT_SUBSCRIBE_TOPIC) {
            cJSON* root = cJSON_Parse(payload.c_str());
            if (root != nullptr) {
                cJSON* data_type = cJSON_GetObjectItemCaseSensitive(root, "data_type");
                if (strcmp(data_type->valuestring, "HumanDectectedMonitor") == 0) {
                    cJSON* data_json = cJSON_GetObjectItemCaseSensitive(root, "data");
                    if (cJSON_IsObject(data_json)) {
                        cJSON* x_point = cJSON_GetObjectItemCaseSensitive(data_json, "x_point");
                        if (x_point->valuedouble < -0.22) {
                            uart_write_bytes(UART_NUM_1, DM12_Speed_36_Position_60, sizeof(DM12_Speed_36_Position_60));
                        } else if (x_point->valuedouble > 0.22) {
                            uart_write_bytes(UART_NUM_1, DM12_Speed_36_Position_120, sizeof(DM12_Speed_36_Position_120));
                            auto& app = Application::GetInstance();
                            app.protocol_->Myself_SendText("有人!");
                        } else {
                            uart_write_bytes(UART_NUM_1, DM12_Speed_18_Position_90, sizeof(DM12_Speed_18_Position_90));
                        }
                    }
                }
                cJSON_Delete(root);
            }
        }
    });

    if (!mqtt_instance_->Connect(MQTT_ADDRESS, MQTT_PORT, mac_, MQTT_USERNAME, MQTT_PASSWORD)) {
        ESP_LOGE(TAG, "Failed to connect to endpoint!");
    } else {
        mqtt_instance_->Subscribe(MQTT_SUBSCRIBE_TOPIC);
        // mqtt_instance_->Subscribe(mqtt_publish_topic_mcp_mobileChassis_);
    }
}

void ServoControlBoard::mqtt_publish(const std::string& text) {
    // cJSON* root = cJSON_CreateObject();
    // cJSON_AddStringToObject(root, "device_id", mac_);
    // cJSON_AddStringToObject(root, "msg", text.c_str());
    // char* json_str = cJSON_Print(root);
    // std::string str(json_str);

    std::string payload = "{\"device_id\":\"" + mac_ + "\",\"msg\":\"" + text + "\"}";
    if (!mqtt_instance_->Publish(mqtt_publish_topic_log_deviceState_, payload)) {
        ESP_LOGE(TAG, "Failed to publish message: %s", text.c_str());
    }

    // mqtt_instance_->Publish(mqtt_publish_topic_mcp_mobileChassis_, payload);
    // free(json_str);
    // cJSON_Delete(root);
}

void ServoControlBoard::init_eyelid() {
    auto eyelid_func = [](void* arg) -> void {
        while (true) {
            uart_write_bytes(UART_NUM_1, DM5_Speed_90_Position_65, sizeof(DM5_Speed_90_Position_65));
            uart_write_bytes(UART_NUM_1, DM6_Speed_90_Position_115, sizeof(DM6_Speed_90_Position_115));
            vTaskDelay(pdMS_TO_TICKS(500));
            uart_write_bytes(UART_NUM_1, DM5_Speed_90_Position_90, sizeof(DM5_Speed_90_Position_90));
            uart_write_bytes(UART_NUM_1, DM6_Speed_90_Position_90, sizeof(DM6_Speed_90_Position_90));
            vTaskDelay(pdMS_TO_TICKS(10 * 1000));
        }
        vTaskDelete(NULL);
    };
    xTaskCreate((TaskFunction_t)eyelid_func, "eyelid_func", 2048, NULL, 2, &faceAction_eyelid_handle_);
}

void ServoControlBoard::init_eyeball() {
    auto eyeball_func = [](void* arg) -> void {
        while (true) {
            uart_write_bytes(UART_NUM_1, DM4_Speed_90_Position_110, sizeof(DM4_Speed_90_Position_110));
            vTaskDelay(pdMS_TO_TICKS(500)); // Unit: ms
            uart_write_bytes(UART_NUM_1, DM4_Speed_90_Position_90, sizeof(DM4_Speed_90_Position_90));
            vTaskDelay(pdMS_TO_TICKS(500)); // Unit: ms
            uart_write_bytes(UART_NUM_1, DM4_Speed_90_Position_70, sizeof(DM4_Speed_90_Position_70));
            vTaskDelay(pdMS_TO_TICKS(500)); // Unit: ms
            uart_write_bytes(UART_NUM_1, DM4_Speed_90_Position_90, sizeof(DM4_Speed_90_Position_90));
            vTaskDelay(pdMS_TO_TICKS(15 * 1000)); // Unit: ms
        }
        vTaskDelete(NULL);
    };
    xTaskCreate((TaskFunction_t)eyeball_func, "eyeball_func", 2048, NULL, 2, &faceAction_eyeball_handle_);
}

void ServoControlBoard::init_chin() {
    auto chin_func = [](void* arg) -> void {
        while (true) {
            uart_write_bytes(UART_NUM_1, DM0_Speed_90_Position_100, sizeof(DM0_Speed_90_Position_100));
            vTaskDelay(pdMS_TO_TICKS(200)); // Unit: ms
            uart_write_bytes(UART_NUM_1, DM0_Speed_90_Position_90, sizeof(DM0_Speed_90_Position_90));
            vTaskDelay(pdMS_TO_TICKS(200)); // Unit: ms
            uart_write_bytes(UART_NUM_1, DM0_Speed_90_Position_95, sizeof(DM0_Speed_90_Position_95));
            vTaskDelay(pdMS_TO_TICKS(200)); // Unit: ms
            uart_write_bytes(UART_NUM_1, DM0_Speed_90_Position_90, sizeof(DM0_Speed_90_Position_90));
            vTaskDelay(pdMS_TO_TICKS(200)); // Unit: ms
        }
        vTaskDelete(NULL);
    };
    xTaskCreate((TaskFunction_t)chin_func, "chin_func", 2048, NULL, 2, &faceAction_chin_handle_);
}

void ServoControlBoard::init_header() {
    init_eyelid();
    init_eyeball();
    init_chin();
    vTaskSuspend(faceAction_chin_handle_);
    uart_write_bytes(UART_NUM_1, DM0_Speed_18_Position_90, sizeof(DM0_Speed_18_Position_90));
    uart_write_bytes(UART_NUM_1, DM12_Speed_18_Position_90, sizeof(DM12_Speed_18_Position_90));
    ESP_LOGI(TAG, "::init_header() -> Execution successful!");
}

void ServoControlBoard::neutral() {
    uart_write_bytes(UART_NUM_1, DM0_Speed_90_Position_75, sizeof(DM0_Speed_90_Position_75));
    uart_write_bytes(UART_NUM_1, DM1_Speed_18_Position_80, sizeof(DM1_Speed_18_Position_80));
    uart_write_bytes(UART_NUM_1, DM2_Speed_18_Position_100, sizeof(DM2_Speed_18_Position_100));
    if (faceAction_eyelid_handle_ != nullptr && faceAction_eyeball_handle_ != nullptr) {
        vTaskResume(faceAction_eyelid_handle_);
        vTaskResume(faceAction_eyeball_handle_);
    }
}

void ServoControlBoard::speaking_state() {
    uart_write_bytes(UART_NUM_1, DM1_Speed_90_Position_60, sizeof(DM1_Speed_90_Position_60));
    uart_write_bytes(UART_NUM_1, DM2_Speed_90_Position_120, sizeof(DM2_Speed_90_Position_120));
}

void ServoControlBoard::sleepy() {
    vTaskSuspend(faceAction_eyelid_handle_);
    vTaskSuspend(faceAction_eyeball_handle_);

    uart_write_bytes(UART_NUM_1, DM0_Speed_18_Position_90, sizeof(DM0_Speed_18_Position_90));
    uart_write_bytes(UART_NUM_1, DM1_Speed_18_Position_90, sizeof(DM1_Speed_18_Position_90));
    uart_write_bytes(UART_NUM_1, DM2_Speed_18_Position_90, sizeof(DM2_Speed_18_Position_90));

    uart_write_bytes(UART_NUM_1, DM4_Speed_90_Position_90, sizeof(DM4_Speed_90_Position_90));
    uart_write_bytes(UART_NUM_1, DM5_Speed_90_Position_65, sizeof(DM5_Speed_90_Position_65));
    uart_write_bytes(UART_NUM_1, DM6_Speed_90_Position_115, sizeof(DM6_Speed_90_Position_115));

    uart_write_bytes(UART_NUM_1, DM12_Speed_18_Position_90, sizeof(DM12_Speed_18_Position_90));
}