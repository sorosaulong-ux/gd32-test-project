//==========================================================
// 文件名: onenet.c
// 功能:   处理与移动 OneNET 物联网平台（新版物模型）的加密登录、
//         数据上报、指令下发接收与解析回复。
//==========================================================

//单片机头文件
#include "gd32f30x.h"

//网络设备
#include "esp8266.h"

//协议文件
#include "onenet.h"
#include "MqttKit.h"

//算法 (用于生成 OneNET 登录鉴权 Token)
#include "base64.h"
#include "hmac_sha1.h"

//硬件驱动
#include "usart.h"
#include "delay.h"
#include "buzzer.h"   

//C库
#include <string.h>
#include <stdio.h>
#include "cJSON.h"

//==========================================================
// 用户的 OneNET 产品与设备信息 (请严格保密设备密钥)
//==========================================================
#define PROID			"koQ9729Ag8"                                        // 产品 ID
#define ACCESS_KEY		"WFNyVHNTbU50c3ZzNWtqcHBIVkN6dEs2cERNRkIwQ3A="      // 设备密钥 (Device Key)
#define DEVICE_NAME		"car1"                                              // 设备名称

char devid[16];
char key[48];
 
// 引用底层的 ESP8266 接收缓冲区
extern unsigned char esp8266_buf[512];

/*
************************************************************
*	函数名称:	OTA_UrlEncode
*	函数功能:	对签名进行 URL 编码
*	说明:		OneNET 要求的底层格式化转换，将特殊字符转为百分号编码
************************************************************
*/
static unsigned char OTA_UrlEncode(char *sign)
{
	char sign_t[40];
	unsigned char i = 0, j = 0;
	unsigned char sign_len = strlen(sign);
	
	if(sign == (void *)0 || sign_len < 28) return 1;
	for(; i < sign_len; i++) { sign_t[i] = sign[i]; sign[i] = 0; }
	sign_t[i] = 0;
	for(i = 0, j = 0; i < sign_len; i++)
	{
		switch(sign_t[i])
		{
			case '+': strcat(sign + j, "%2B"); j += 3; break;
			case ' ': strcat(sign + j, "%20"); j += 3; break;
			case '/': strcat(sign + j, "%2F"); j += 3; break;
			case '?': strcat(sign + j, "%3F"); j += 3; break;
			case '%': strcat(sign + j, "%25"); j += 3; break;
			case '#': strcat(sign + j, "%23"); j += 3; break;
			case '&': strcat(sign + j, "%26"); j += 3; break;
			case '=': strcat(sign + j, "%3D"); j += 3; break;
			default: sign[j] = sign_t[i]; j++; break;
		}
	}
	sign[j] = 0; return 0;
}

/*
************************************************************
*	函数名称:	OneNET_Authorization
*	函数功能:	生成 OneNET 连接所需的鉴权 Token (Authorization 字段)
*	说明:		使用 HMAC-SHA1 算法和 Base64 对密钥进行加密
************************************************************
*/
#define METHOD		"sha1"
static unsigned char OneNET_Authorization(char *ver, char *res, unsigned int et, char *access_key, char *dev_name, char *authorization_buf, unsigned short authorization_buf_len, _Bool flag)
{
	size_t olen = 0;
	char sign_buf[64], hmac_sha1_buf[64], access_key_base64[64], string_for_signature[72];					
	
	if(ver == (void *)0 || res == (void *)0 || et < 1564562581 || access_key == (void *)0 || authorization_buf == (void *)0 || authorization_buf_len < 120) return 1;
	
    // 1. Base64 解码 AccessKey
    memset(access_key_base64, 0, sizeof(access_key_base64));
	BASE64_Decode((unsigned char *)access_key_base64, sizeof(access_key_base64), &olen, (unsigned char *)access_key, strlen(access_key));
	
    // 2. 拼接签名字符串
    memset(string_for_signature, 0, sizeof(string_for_signature));
	if(flag) snprintf(string_for_signature, sizeof(string_for_signature), "%d\n%s\nproducts/%s\n%s", et, METHOD, res, ver);
	else snprintf(string_for_signature, sizeof(string_for_signature), "%d\n%s\nproducts/%s/devices/%s\n%s", et, METHOD, res, dev_name, ver);
	
    // 3. HMAC-SHA1 加密
    memset(hmac_sha1_buf, 0, sizeof(hmac_sha1_buf));
	hmac_sha1((unsigned char *)access_key_base64, strlen(access_key_base64), (unsigned char *)string_for_signature, strlen(string_for_signature), (unsigned char *)hmac_sha1_buf);
	
    // 4. Base64 编码并进行 URL 转换
    olen = 0; memset(sign_buf, 0, sizeof(sign_buf));
	BASE64_Encode((unsigned char *)sign_buf, sizeof(sign_buf), &olen, (unsigned char *)hmac_sha1_buf, strlen(hmac_sha1_buf));
	OTA_UrlEncode(sign_buf);
	
    // 5. 格式化输出最终的 Token
    if(flag) snprintf(authorization_buf, authorization_buf_len, "version=%s&res=products%%2F%s&et=%d&method=%s&sign=%s", ver, res, et, METHOD, sign_buf);
	else snprintf(authorization_buf, authorization_buf_len, "version=%s&res=products%%2F%s%%2Fdevices%%2F%s&et=%d&method=%s&sign=%s", ver, res, dev_name, et, METHOD, sign_buf);
	
    return 0;
}

/*
************************************************************
*	函数名称:	OneNet_DevLink
*	函数功能:	与 OneNET 平台建立 MQTT 连接
*	返回参数:	0: 登录成功   1: 登录失败
************************************************************
*/
_Bool OneNet_DevLink(void)
{
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};					
	unsigned char *dataPtr; 
    char authorization_buf[160]; 
    _Bool status = 1;
	
    // 1. 传入参数生成 Token (有效时间设置为 1956499200，长期有效)
	OneNET_Authorization("2018-10-31", PROID, 1956499200, ACCESS_KEY, DEVICE_NAME, authorization_buf, sizeof(authorization_buf), 0);
	
    // 2. 组装 MQTT 的 CONNECT 报文
	if(MQTT_PacketConnect(PROID, authorization_buf, DEVICE_NAME, 256, 1, MQTT_QOS_LEVEL0, NULL, NULL, 0, &mqttPacket) == 0)
	{
        UsartPrintf(USART_DEBUG, "--> 正在向平台发送 MQTT 登录请求包...\r\n");
		ESP8266_SendData(mqttPacket._data, mqttPacket._len);			
		
        // 3. 阻塞等待平台下发的 CONNACK 回复包 (最多等待约 1.25秒)
		dataPtr = ESP8266_GetIPD(250);									
		if(dataPtr != NULL)
		{
			if(MQTT_UnPacketRecv(dataPtr) == MQTT_PKT_CONNACK)
			{
                // 4. 解析回复包的状态码 (已删除冗余调试信息)
				switch(MQTT_UnPacketConnectAck(dataPtr))
				{
					case 0: UsartPrintf(USART_DEBUG, "Tips: 登录成功\r\n"); status = 0; break;
					default: UsartPrintf(USART_DEBUG, "ERR: 登录失败\r\n"); break;
				}
			}
		}
        
        // 5. 释放报文内存，防止内存泄漏
		MQTT_DeleteBuffer(&mqttPacket);								
	}
    else
    {
        UsartPrintf(USART_DEBUG, "ERR: MQTT 封包内存分配失败！\r\n");
    }
    
	return status;
}

/*
************************************************************
*	函数名称:	OneNet_FillBuf
*	函数功能:	按照 OneNET 物模型的标准 JSON 格式打包数据
*	返回参数:	打包后的数据长度
************************************************************
*/
unsigned char OneNet_FillBuf(char *buf)
{
	char text[64];

    // 1. 拼接 JSON 头部 (id 可以任意，通常用作消息追踪)
	memset(text, 0, sizeof(text));
	strcpy(buf, "{\"id\":\"123\",\"version\":\"1.0\",\"params\":{");
	
    // 2. 拼接具体的属性值 (Buzzer 是云端定义的标识符)
	memset(text, 0, sizeof(text));
	sprintf(text, "\"Buzzer\":{\"value\":%s}", BUZZER_Status == BUZZER_ON ? "true" : "false");
	strcat(buf, text);
	
    // 3. 拼接 JSON 尾部
	strcat(buf, "}}");
	return strlen(buf);
}

/*
************************************************************
*	函数名称:	OneNet_SendData
*	函数功能:	向 OneNET 平台上报设备属性数据 (Post)
************************************************************
*/
void OneNet_SendData(void)
{
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};												
	char buf[256]; 
	char topic_buf[128];
	short body_len = 0;
	
    // 1. 获取打包好的 JSON 数据
	memset(buf, 0, sizeof(buf));
	body_len = OneNet_FillBuf(buf);																	
	
	if(body_len)
	{
        // 2. 拼接物模型标准上报 Topic: $sys/{pid}/{device-name}/thing/property/post
		snprintf(topic_buf, sizeof(topic_buf), "$sys/%s/%s/thing/property/post", PROID, DEVICE_NAME);
		
        // 3. 打包为 MQTT PUBLISH 报文并发送
        if(MQTT_PacketPublish(MQTT_PUBLISH_ID, topic_buf, buf, body_len, MQTT_QOS_LEVEL0, 0, 1, &mqttPacket) == 0)				
		{
			ESP8266_SendData(mqttPacket._data, mqttPacket._len);									
			MQTT_DeleteBuffer(&mqttPacket);															
		}
	}
}

/*
************************************************************
*	函数名称:	OneNET_Subscribe
*	函数功能:	订阅 OneNET 平台的属性设置 Topic
* 说明:       只有订阅了这个主题，设备才能收到云端下发的控制指令
************************************************************
*/
void OneNET_Subscribe(void)
{
	MQTT_PACKET_STRUCTURE mqtt_packet = {NULL, 0, 0, 0};						
	char topic_buf[56]; 
    const char *topic = topic_buf;
    
    // 订阅的 Topic: $sys/{pid}/{device-name}/thing/property/set
	snprintf(topic_buf, sizeof(topic_buf), "$sys/%s/%s/thing/property/set", PROID, DEVICE_NAME);
	
    if(MQTT_PacketSubscribe(MQTT_SUBSCRIBE_ID, MQTT_QOS_LEVEL0, &topic, 1, &mqtt_packet) == 0)
	{
		ESP8266_SendData(mqtt_packet._data, mqtt_packet._len);					
		MQTT_DeleteBuffer(&mqtt_packet);										
	}
}

/*
************************************************************
*	函数名称:	OneNet_RevPro
*	函数功能:	解析云端下发的 MQTT 报文并执行硬件控制
* 入口参数:   cmd: 接收到的原始报文数据指针
************************************************************
*/
void OneNet_RevPro(unsigned char *cmd)
{
	char *req_payload = NULL, *cmdid_topic = NULL;
	unsigned short topic_len = 0, req_len = 0;
	unsigned char qos = 0, type = 0;
	static unsigned short pkt_id = 0;
	short result = 0;

	cJSON *raw_json, *params_json, *buzzer_json, *id_json;
	char msg_id[32] = {0}; 
	char reply_topic[128];
	char reply_payload[128];
	MQTT_PACKET_STRUCTURE mqtt_reply = {NULL, 0, 0, 0};
	
    // 1. 判断报文类型
	type = MQTT_UnPacketRecv(cmd);
	switch(type)
	{
        // 平台下发的数据 (通常是控制指令)
		case MQTT_PKT_PUBLISH:
            // 拆包提取 Payload (有效载荷，即 JSON 字符串)
			result = MQTT_UnPacketPublish(cmd, &cmdid_topic, &topic_len, &req_payload, &req_len, &qos, &pkt_id);
			if(result == 0)
			{
				req_payload[req_len] = '\0'; 
				UsartPrintf(USART_DEBUG, "\r\n[1] Recv Payload: %s\r\n", req_payload);
				
                // 2. 使用 cJSON 解析接收到的 JSON 字符串
				raw_json = cJSON_Parse(req_payload);
				if(raw_json != NULL)
                {
                    // 3. 提取 msg_id (用于给平台回复确认包时一一对应)
                    id_json = cJSON_GetObjectItem(raw_json, "id");
                    if(id_json != NULL)
                    {
                        if(id_json->type == cJSON_String && id_json->valuestring != NULL)
                            strncpy(msg_id, id_json->valuestring, sizeof(msg_id) - 1);
                        else if(id_json->type == cJSON_Number || id_json->type == cJSON_False || id_json->type == cJSON_True)
                            snprintf(msg_id, sizeof(msg_id), "%d", id_json->valueint);
                    }
                    UsartPrintf(USART_DEBUG, "[2] Parsed msg_id: [%s]\r\n", msg_id);

                    // 4. 解析控制参数并操作硬件
                    params_json = cJSON_GetObjectItem(raw_json, "params");
                    if(params_json != NULL)
                    {
                        // 查找 "Buzzer" 键
                        buzzer_json = cJSON_GetObjectItem(params_json, "Buzzer");
                        if(buzzer_json != NULL)
                        {
                            // 兼容多种数值类型的布尔判断
                            if(buzzer_json->type == cJSON_True || buzzer_json->type == 1 || buzzer_json->type == 2 || buzzer_json->valueint == 1) 
                                BUZZER_Set(BUZZER_ON);   
                            else 
                                BUZZER_Set(BUZZER_OFF);  
                            
                            UsartPrintf(USART_DEBUG, "[3] BUZZER Action Executed!\r\n");
                        }
                    }
                    
                    // 5. 【极其关键】提取完毕后立即释放 cJSON 内存，防止堆内存溢出
                    cJSON_Delete(raw_json);
                    
                    // 6. 硬件执行完毕，向云端发送 ACK 确认包 (set_reply)
                    if(strlen(msg_id) > 0)
                    {
                        snprintf(reply_topic, sizeof(reply_topic), "$sys/%s/%s/thing/property/set_reply", PROID, DEVICE_NAME);
                        snprintf(reply_payload, sizeof(reply_payload), "{\"id\":\"%s\",\"code\":200,\"msg\":\"success\"}", msg_id);
                        
                        UsartPrintf(USART_DEBUG, "[4] Ready to send ACK: %s\r\n", reply_payload);

                        // 打包并发送回复
                        if(MQTT_PacketPublish(MQTT_PUBLISH_ID, reply_topic, reply_payload, strlen(reply_payload), MQTT_QOS_LEVEL0, 0, 1, &mqtt_reply) == 0)				
                        {
                            ESP8266_SendData(mqtt_reply._data, mqtt_reply._len);									
                            MQTT_DeleteBuffer(&mqtt_reply);	
                            UsartPrintf(USART_DEBUG, "[5] --> ACK Sent Successfully!\r\n");
                        }
                        else
                        {
                            UsartPrintf(USART_DEBUG, "[!] --> Still Failed. Heap size is too small!\r\n");
                        }
                    }
                }
			}
        break;
			
        // 我们发送的数据，平台返回的确认包
		case MQTT_PKT_PUBACK:
			if(MQTT_UnPacketPublishAck(cmd) == 0) UsartPrintf(USART_DEBUG, "Tips: MQTT Publish Send OK\r\n");
		break;
        
        // 我们发送的订阅请求，平台返回的确认包
		case MQTT_PKT_SUBACK:
			if(MQTT_UnPacketSubscribe(cmd) == 0) UsartPrintf(USART_DEBUG, "Tips: MQTT Subscribe OK\r\n");
		break;
        
		default: result = -1; break;
	}
	
    // 清空底层接收缓冲区
	ESP8266_Clear();
	if(result == -1) return;
	
    // 释放由 MQTT_UnPacketPublish 函数内部分配的内存
	if(type == MQTT_PKT_CMD || type == MQTT_PKT_PUBLISH)
	{
		MQTT_FreeBuffer(cmdid_topic);
		MQTT_FreeBuffer(req_payload);
	}
}