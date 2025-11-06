#include <cJSON.h>
#include <driver/uart.h>
#include <esp_log.h>

#include "esp_mac.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "mcp_server.h"
#include "servo_control_board.h"

static const char* TAG = "ServoControlBoard";

ServoControlBoard::ServoControlBoard(void) {
    uart1_init();
    add_mcp_tools();
}

void ServoControlBoard::uart1_init() {
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

void ServoControlBoard::add_mcp_tools() {
    auto& mcp_server = McpServer::GetInstance();
    mcp_server.AddTool("self.face_action.smiled", "脸部_动作->微笑", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
        uart_write_bytes(UART_NUM_1, DM1_Speed_20_Position_60, sizeof(DM1_Speed_20_Position_60));
        uart_write_bytes(UART_NUM_1, DM2_Speed_20_Position_120, sizeof(DM2_Speed_20_Position_120));
        return true;
    });
    mcp_server.AddTool("self.face_action.grimace", "脸部_动作->做个鬼脸", PropertyList(),
                       [this](const PropertyList& properties) -> ReturnValue {
                           uart_write_bytes(UART_NUM_1, DM1_Speed_20_Position_60, sizeof(DM1_Speed_20_Position_60));
                           uart_write_bytes(UART_NUM_1, DM5_Speed_20_Position_110, sizeof(DM5_Speed_20_Position_110));
                           vTaskDelay(pdMS_TO_TICKS(1000));
                           uart_write_bytes(UART_NUM_1, DM2_Speed_20_Position_120, sizeof(DM2_Speed_20_Position_120));
                           uart_write_bytes(UART_NUM_1, DM6_Speed_20_Position_70, sizeof(DM6_Speed_20_Position_70));
                           return true;
                       });
}

void ServoControlBoard::add_esp_now() {
    // 获取当前AP信息
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

    // 初始化 ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    // 注册接收回调
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
                        uart_write_bytes(UART_NUM_1, DM12_Speed_3_Position_70, sizeof(DM12_Speed_3_Position_70));
                    } else if (x_point->valuedouble > 0.22) {
                        uart_write_bytes(UART_NUM_1, DM12_Speed_3_Position_110, sizeof(DM12_Speed_3_Position_110));
                    } else {
                        uart_write_bytes(UART_NUM_1, DM12_Speed_3_Position_90, sizeof(DM12_Speed_3_Position_90));
                    }
                }
            }
            cJSON_Delete(object);
        } else {
            ESP_LOGW(TAG, ">>> Received invalid data length: %d", data_len);
        }

        // 判断是否为广播包
        // bool is_broadcast = (memcmp(esp_now_info->des_addr, "\xFF\xFF\xFF\xFF\xFF\xFF", 6) == 0);
        // if (is_broadcast) {
        //     ESP_LOGI(TAG, ">>> This is a BROADCAST packet!");
        // }
    });
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP NOW 注册接收回调函数失败,错误码: 0x%X", ret);
    }

    // 获取并打印本机 MAC 地址(用于发送端配置)
    uint8_t mac_addr[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac_addr);
    ESP_LOGW(TAG, "=== My MAC Address: %02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
             mac_addr[5]);
}

void ServoControlBoard::nictation() {
    auto nictation_func = [](void* arg) -> void {
        while (true) {
            uart_write_bytes(UART_NUM_1, DM5_Speed_20_Position_50, sizeof(DM5_Speed_20_Position_50));
            uart_write_bytes(UART_NUM_1, DM6_Speed_10_Position_100, sizeof(DM6_Speed_10_Position_100));
            vTaskDelay(pdMS_TO_TICKS(500));
            uart_write_bytes(UART_NUM_1, DM5_Speed_20_Position_90, sizeof(DM5_Speed_20_Position_90));
            uart_write_bytes(UART_NUM_1, DM6_Speed_10_Position_90, sizeof(DM6_Speed_10_Position_90));
            vTaskDelay(pdMS_TO_TICKS(10 * 1000));
        }
        vTaskDelete(NULL);
    };
    xTaskCreate((TaskFunction_t)nictation_func, "nictation_func", 2048, NULL, 2, &faceAction_nictation_handle_);
}

void ServoControlBoard::eyeball() {
    auto eye_func = [](void* arg) -> void {
        while (true) {
            uart_write_bytes(UART_NUM_1, DM4_Speed_20_Position_120, sizeof(DM4_Speed_20_Position_120));
            vTaskDelay(pdMS_TO_TICKS(500)); // Unit: ms
            uart_write_bytes(UART_NUM_1, DM4_Speed_20_Position_90, sizeof(DM4_Speed_20_Position_90));
            vTaskDelay(pdMS_TO_TICKS(500)); // Unit: ms
            uart_write_bytes(UART_NUM_1, DM4_Speed_20_Position_60, sizeof(DM4_Speed_20_Position_60));
            vTaskDelay(pdMS_TO_TICKS(500)); // Unit: ms
            uart_write_bytes(UART_NUM_1, DM4_Speed_20_Position_90, sizeof(DM4_Speed_20_Position_90));
            vTaskDelay(pdMS_TO_TICKS(15 * 1000)); // Unit: ms
        }
        vTaskDelete(NULL);
    };
    xTaskCreate((TaskFunction_t)eye_func, "eye_func", 2048, NULL, 2, &faceAction_eyeball_handle_);
}

void ServoControlBoard::speaking() {
    auto chin_func = [](void* arg) -> void {
        while (true) {
            uart_write_bytes(UART_NUM_1, DM0_Speed_10_Position_110, sizeof(DM0_Speed_20_Position_110));
            vTaskDelay(pdMS_TO_TICKS(200)); // Unit: ms
            uart_write_bytes(UART_NUM_1, DM0_Speed_10_Position_90, sizeof(DM0_Speed_20_Position_90));
            vTaskDelay(pdMS_TO_TICKS(200)); // Unit: ms
            uart_write_bytes(UART_NUM_1, DM0_Speed_10_Position_100, sizeof(DM0_Speed_10_Position_100));
            vTaskDelay(pdMS_TO_TICKS(200)); // Unit: ms
            uart_write_bytes(UART_NUM_1, DM0_Speed_10_Position_90, sizeof(DM0_Speed_20_Position_90));
            vTaskDelay(pdMS_TO_TICKS(200)); // Unit: ms
        }
        vTaskDelete(NULL);
    };
    xTaskCreate((TaskFunction_t)chin_func, "chin_func", 2048, NULL, 2, &faceAction_chin_handle_);
}

// void ServoControlBoard::head() {
//     auto head_func = [](void* arg) -> void {
//         while (true) {
//             uart_write_bytes(UART_NUM_1, DM12_Speed_3_Position_90, sizeof(DM12_Speed_3_Position_90));
//             vTaskDelay(pdMS_TO_TICKS(1000));
//             uart_write_bytes(UART_NUM_1, DM12_Speed_3_Position_70, sizeof(DM12_Speed_3_Position_70));
//             vTaskDelay(pdMS_TO_TICKS(1000));
//             uart_write_bytes(UART_NUM_1, DM12_Speed_3_Position_110, sizeof(DM12_Speed_3_Position_110));
//             vTaskDelay(pdMS_TO_TICKS(1000));
//             uart_write_bytes(UART_NUM_1, DM12_Speed_3_Position_90, sizeof(DM12_Speed_3_Position_90));
//             vTaskDelay(pdMS_TO_TICKS(20 * 1000));
//         }
//         vTaskDelete(NULL);
//     };
//     xTaskCreate((TaskFunction_t)head_func, "head_func", 2048, NULL, 2, &speaking_handle_);
// }

void ServoControlBoard::neutral() {
    uart_write_bytes(UART_NUM_1, DM0_Speed_20_Position_90, sizeof(DM0_Speed_20_Position_90));
    uart_write_bytes(UART_NUM_1, DM1_Speed_10_Position_90, sizeof(DM1_Speed_10_Position_90));
    uart_write_bytes(UART_NUM_1, DM2_Speed_10_Position_90, sizeof(DM2_Speed_10_Position_90));

    // uart_write_bytes(UART_NUM_1, DM4_Speed_20_Position_90, sizeof(DM4_Speed_20_Position_90));
    // uart_write_bytes(UART_NUM_1, DM5_Speed_20_Position_90, sizeof(DM5_Speed_20_Position_90));
    // uart_write_bytes(UART_NUM_1, DM6_Speed_10_Position_90, sizeof(DM6_Speed_10_Position_90));

    // uart_write_bytes(UART_NUM_1, DM12_Speed_3_Position_90, sizeof(DM12_Speed_3_Position_90));

    vTaskResume(faceAction_nictation_handle_);
    vTaskResume(faceAction_eyeball_handle_);
}

void ServoControlBoard::smiled() {
    vTaskSuspend(faceAction_nictation_handle_);
    uart_write_bytes(UART_NUM_1, DM5_Speed_20_Position_50, sizeof(DM5_Speed_20_Position_50));
    uart_write_bytes(UART_NUM_1, DM6_Speed_10_Position_100, sizeof(DM6_Speed_10_Position_100));

    uart_write_bytes(UART_NUM_1, DM1_Speed_20_Position_60, sizeof(DM1_Speed_20_Position_60));
    uart_write_bytes(UART_NUM_1, DM2_Speed_20_Position_120, sizeof(DM2_Speed_20_Position_120));
}

void ServoControlBoard::sleepy() {
    vTaskSuspend(faceAction_nictation_handle_);
    vTaskSuspend(faceAction_eyeball_handle_);

    uart_write_bytes(UART_NUM_1, DM0_Speed_20_Position_70, sizeof(DM0_Speed_20_Position_70));
    uart_write_bytes(UART_NUM_1, DM1_Speed_20_Position_90, sizeof(DM1_Speed_20_Position_90));
    uart_write_bytes(UART_NUM_1, DM2_Speed_20_Position_90, sizeof(DM2_Speed_20_Position_90));

    uart_write_bytes(UART_NUM_1, DM4_Speed_20_Position_90, sizeof(DM4_Speed_20_Position_90));
    uart_write_bytes(UART_NUM_1, DM5_Speed_20_Position_50, sizeof(DM5_Speed_20_Position_50));
    uart_write_bytes(UART_NUM_1, DM6_Speed_20_Position_130, sizeof(DM6_Speed_20_Position_130));

    uart_write_bytes(UART_NUM_1, DM12_Speed_3_Position_90, sizeof(DM12_Speed_3_Position_90));
}