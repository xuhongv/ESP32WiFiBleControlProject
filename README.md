# 微信小程序蓝牙+WiFi双控制ESP32应用示范

<p align="center">
  <img src="https://img.alicdn.com/imgextra/i1/2922621297/O1CN01XgJTKh1LS4H0ugNWc_!!2922621297.png"  alt="Banner"  width="520px" height="414px"  />
</p>
<p align="center">
  <img src="images/aithinker-mini.jpg"  alt="Banner" width="270px" height="580px"  />
</p>



# 一、前言

目前市场上越来越火的 Combo 方案（**Ble+WiFi**），比如平头哥的TG7100C方案、乐鑫的ESP32、博通的BK7251等，如何高效使用蓝牙和wifi通讯，已经成为了必然的趋势，于是乎，做了个这样快速入门的demo给各位，奉献于物联网；

本项目适合的模组有：

| 模组                   | 链接                  |
| ---------------------- | --------------------- |
| 安信可ESP32-S模组      | http://yli08.cn/wjy03 |
| 安信可ESP32-SL模组     | http://yli08.cn/IKalA |
| 安信可ESP32-C3系列模组 | 联系商务              |

本开源工程用到的技术点有：

1. 乐鑫物联网操作框架 esp-idf 的 freeRtos 实时操作系统熟悉，包括任务创建/消息队列/进程间通讯；
2. 微信小程序开发基础，包括MQTT库/低功耗蓝牙API接口使用，包括搜索/连接/通讯；
3. 使用乐鑫封装 RMT 驱动层单线驱动WS2812B，实现彩虹等效果；
4. 对ESP32/C3芯片的外设开发熟悉，对BLE API接口使用熟悉，包括自定义广播/名字/自定义UUID；

# 二、核心代码

## 2.1 蓝牙控制

设置蓝牙广播名字

```c
esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
```

设置服务UUID

```c
 gl_profile_tab[0].service_id.is_primary = true;
 gl_profile_tab[0].service_id.id.inst_id = 0x00;
 gl_profile_tab[0].service_id.id.uuid.len = ESP_UUID_LEN_16;
 gl_profile_tab[0].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;
```

主动通知上位机数据发生改动：

```c
 case ESP_GATTS_READ_EVT:
    {
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 3;
        rsp.attr_value.value[0] = red;
        rsp.attr_value.value[1] = green;
        rsp.attr_value.value[2] = blue;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    }
```

上位机主动发送数据到此并做出对应的处理：

```c
 case ESP_GATTS_WRITE_EVT:
    {
        if (!param->write.is_prep)
        {
            ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
            //发送数据到队列
            struct __User_data *pTmper;
            sprintf(user_data.allData, "{\"red\":%d,\"green\":%d,\"blue\":%d}", param->write.value[0], param->write.value[1], param->write.value[2]);
            pTmper = &user_data;
            user_data.dataLen = strlen(user_data.allData);
            xQueueSend(ParseJSONQueueHandler, (void *)&pTmper, portMAX_DELAY);

            ESP_LOGI(GATTS_TAG, "%02x %02x %02x ", param->write.value[0], param->write.value[1], param->write.value[2]);
        }
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;
    }
```

## 2.2 WiFi控制

设置MQTT远程连接的参数

```c
/* 
 * @Description: MQTT参数连接的配置
 * @param: 
 * @return: 
*/
void TaskXMqttRecieve(void *p)
{
    //连接的配置参数
    esp_mqtt_client_config_t mqtt_cfg = {
        .host = "www.xuhong.com",  //连接的域名 ，请务必修改为您的
        .port = 1883,              //端口，请务必修改为您的
        .username = "admin",       //用户名，请务必修改为您的
        .password = "xuhong123456",   //密码，请务必修改为您的
        .client_id = deviceUUID,
        .event_handle = MqttCloudsCallBack, //设置回调函数
        .keepalive = 120,                   //心跳
        .disable_auto_reconnect = false,    //开启自动重连
        .disable_clean_session = false,     //开启 清除会话
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);

    vTaskDelete(NULL);
}
```

服务器下发的处理数据，送往消息队列处理：

```
    //服务器下发消息到本地成功接收回调
    case MQTT_EVENT_DATA:
    {
        printf("TOPIC=%.*s \r\n", event->topic_len, event->topic);
        printf("DATA=%.*s \r\n\r\n", event->data_len, event->data);
        //发送数据到队列
        struct __User_data *pTmper;
        sprintf(user_data.allData, "%s", event->data);
        pTmper = &user_data;
        user_data.dataLen = event->data_len;
        xQueueSend(ParseJSONQueueHandler, (void *)&pTmper, portMAX_DELAY);
        break;
    }
```

## 2.3 外设驱动

七彩灯WS2812B的驱动代码初始化：

```c
/**
 * @description:  封装一层设置RGB灯效果
 * @param {uint16_t} Red 入参 红色
 * @param {uint16_t} Green 入参 绿色
 * @param {uint16_t} Blue 入参 蓝色
 * @return {*}
 */
void set_rgb(uint16_t Red, uint16_t Green, uint16_t Blue)
{
    for (int i = 0; i < 24; i++)
    {
        strip->set_pixel(strip, i, Red, Green, Blue);
    }
    red = Red;
    green = Green;
    blue = Blue;
    strip->refresh(strip, 10);
}

/**
 * @description: 初始化LED 
 * @param {*}
 * @return {*}
 */
void init_led()
{
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(4, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(24, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip)
    {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));
    set_rgb(0, 254, 0);
}
```

消息队列处理逻辑：

```c
/* 
 * @Description: 解析下发数据的队列逻辑处理
 * @param: null
 * @return: 
*/
void Task_ParseJSON(void *pvParameters)
{
    printf("[SY] Task_ParseJSON_Message creat ... \n");

    while (1)
    {
        struct __User_data *pMqttMsg;

        printf("Task_ParseJSON_Message xQueueReceive wait [%d] ... \n", esp_get_free_heap_size());
        xQueueReceive(ParseJSONQueueHandler, &pMqttMsg, portMAX_DELAY);

        printf("Task_ParseJSON_Message xQueueReceive get [%s] ... \n", pMqttMsg->allData);

        ////首先整体判断是否为一个json格式的数据
        cJSON *pJsonRoot = cJSON_Parse(pMqttMsg->allData);
        //如果是否json格式数据
        if (pJsonRoot == NULL)
        {
            printf("[SY] Task_ParseJSON_Message xQueueReceive not json ... \n");
            goto __cJSON_Delete;
        }

        cJSON *pJSON_Item_Red = cJSON_GetObjectItem(pJsonRoot, "red");
        cJSON *pJSON_Item_Green = cJSON_GetObjectItem(pJsonRoot, "green");
        cJSON *pJSON_Item_Blue = cJSON_GetObjectItem(pJsonRoot, "blue");

        set_rgb(pJSON_Item_Red->valueint, pJSON_Item_Green->valueint, pJSON_Item_Blue->valueint);

    __cJSON_Delete:
        cJSON_Delete(pJsonRoot);
    }
}

```

















