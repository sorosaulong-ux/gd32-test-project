//==========================================================
// �ļ���: onenet.c
// ����:   �������ƶ� OneNET ������ƽ̨���°���ģ�ͣ��ļ��ܵ�¼��
//         �����ϱ���ָ���·�����������ظ���
//==========================================================

// MCU header
#include "gd32a7xx.h"

// device
#include "esp8266.h"

// protocol
#include "onenet.h"
#include "MqttKit.h"

// crypto
#include "base64.h"
#include "hmac_sha1.h"

// hardware
#include "usart_esp.h"
#include "delay.h"
#include "buzzer.h"
#include "key.h"

//C��
#include <string.h>
#include <stdio.h>
#include "cJSON.h"

//==========================================================
// �û��� OneNET ��Ʒ���豸��Ϣ (���ϸ����豸��Կ)
//==========================================================
#define PROID			"koQ9729Ag8"                                        // ��Ʒ ID
#define ACCESS_KEY		"WFNyVHNTbU50c3ZzNWtqcHBIVkN6dEs2cERNRkIwQ3A="      // �豸��Կ (Device Key)
#define DEVICE_NAME		"car1"                                              // �豸����

char devid[16];
char key[48];
 
// ���õײ�� ESP8266 ���ջ�����
extern unsigned char esp8266_buf[512];

/* ── 外部变量: 来自 main.c ── */
extern uint8_t  g_car_lock;
extern uint8_t  g_brake;
extern uint8_t  g_parking_brake;
extern uint8_t  g_system_status;
extern float    g_key_distance;

/*
************************************************************
*	��������:	OTA_UrlEncode
*	��������:	��ǩ������ URL ����
*	˵��:		OneNET Ҫ��ĵײ��ʽ��ת�����������ַ�תΪ�ٷֺű���
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
*	��������:	OneNET_Authorization
*	��������:	���� OneNET ��������ļ�Ȩ Token (Authorization �ֶ�)
*	˵��:		ʹ�� HMAC-SHA1 �㷨�� Base64 ����Կ���м���
************************************************************
*/
#define METHOD		"sha1"
static unsigned char OneNET_Authorization(char *ver, char *res, unsigned int et, char *access_key, char *dev_name, char *authorization_buf, unsigned short authorization_buf_len, _Bool flag)
{
	size_t olen = 0;
	char sign_buf[64], hmac_sha1_buf[64], access_key_base64[64], string_for_signature[72];					
	
	if(ver == (void *)0 || res == (void *)0 || et < 1564562581 || access_key == (void *)0 || authorization_buf == (void *)0 || authorization_buf_len < 120) return 1;
	
    // 1. Base64 ���� AccessKey
    memset(access_key_base64, 0, sizeof(access_key_base64));
	BASE64_Decode((unsigned char *)access_key_base64, sizeof(access_key_base64), &olen, (unsigned char *)access_key, strlen(access_key));
	
    // 2. ƴ��ǩ���ַ���
    memset(string_for_signature, 0, sizeof(string_for_signature));
	if(flag) snprintf(string_for_signature, sizeof(string_for_signature), "%d\n%s\nproducts/%s\n%s", et, METHOD, res, ver);
	else snprintf(string_for_signature, sizeof(string_for_signature), "%d\n%s\nproducts/%s/devices/%s\n%s", et, METHOD, res, dev_name, ver);
	
    // 3. HMAC-SHA1 ����
    memset(hmac_sha1_buf, 0, sizeof(hmac_sha1_buf));
	hmac_sha1((unsigned char *)access_key_base64, strlen(access_key_base64), (unsigned char *)string_for_signature, strlen(string_for_signature), (unsigned char *)hmac_sha1_buf);
	
    // 4. Base64 ���벢���� URL ת��
    olen = 0; memset(sign_buf, 0, sizeof(sign_buf));
	BASE64_Encode((unsigned char *)sign_buf, sizeof(sign_buf), &olen, (unsigned char *)hmac_sha1_buf, strlen(hmac_sha1_buf));
	OTA_UrlEncode(sign_buf);
	
    // 5. ��ʽ��������յ� Token
    if(flag) snprintf(authorization_buf, authorization_buf_len, "version=%s&res=products%%2F%s&et=%d&method=%s&sign=%s", ver, res, et, METHOD, sign_buf);
	else snprintf(authorization_buf, authorization_buf_len, "version=%s&res=products%%2F%s%%2Fdevices%%2F%s&et=%d&method=%s&sign=%s", ver, res, dev_name, et, METHOD, sign_buf);
	
    return 0;
}

/*
************************************************************
*	��������:	OneNet_DevLink
*	��������:	�� OneNET ƽ̨���� MQTT ����
*	���ز���:	0: ��¼�ɹ�   1: ��¼ʧ��
************************************************************
*/
_Bool OneNet_DevLink(void)
{
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};					
	unsigned char *dataPtr; 
    char authorization_buf[160]; 
    _Bool status = 1;
	
    // 1. ����������� Token (��Чʱ������Ϊ 1956499200��������Ч)
	OneNET_Authorization("2018-10-31", PROID, 1956499200, ACCESS_KEY, DEVICE_NAME, authorization_buf, sizeof(authorization_buf), 0);
	
    // 2. ��װ MQTT �� CONNECT ����
	if(MQTT_PacketConnect(PROID, authorization_buf, DEVICE_NAME, 256, 1, MQTT_QOS_LEVEL0, NULL, NULL, 0, &mqttPacket) == 0)
	{
        UsartPrintf(USART_DEBUG, "--> ������ƽ̨���� MQTT ��¼�����...\r\n");
		ESP8266_SendData(mqttPacket._data, mqttPacket._len);			
		
        // 3. �����ȴ�ƽ̨�·��� CONNACK �ظ��� (���ȴ�Լ 1.25��)
		dataPtr = ESP8266_GetIPD(250);									
		if(dataPtr != NULL)
		{
			if(MQTT_UnPacketRecv(dataPtr) == MQTT_PKT_CONNACK)
			{
                // 4. �����ظ�����״̬�� (��ɾ�����������Ϣ)
				switch(MQTT_UnPacketConnectAck(dataPtr))
				{
					case 0: UsartPrintf(USART_DEBUG, "Tips: ��¼�ɹ�\r\n"); status = 0; break;
					default: UsartPrintf(USART_DEBUG, "ERR: ��¼ʧ��\r\n"); break;
				}
			}
		}
        
        // 5. �ͷű����ڴ棬��ֹ�ڴ�й©
		MQTT_DeleteBuffer(&mqttPacket);								
	}
    else
    {
        UsartPrintf(USART_DEBUG, "ERR: MQTT ����ڴ����ʧ�ܣ�\r\n");
    }
    
	return status;
}

/*
************************************************************
*	��������:	OneNet_FillBuf
*	��������:	���� OneNET ��ģ�͵ı�׼ JSON ��ʽ�������
*	���ز���:	���������ݳ���
************************************************************
*/
unsigned char OneNet_FillBuf(char *buf)
{
	char text[64];

	memset(buf, 0, 256);
	strcpy(buf, "{\"id\":\"123\",\"version\":\"1.0\",\"params\":{");
	
	/* Buzzer */
	memset(text, 0, sizeof(text));
	sprintf(text, "\"Buzzer\":{\"value\":%s}", BUZZER_Status == BUZZER_ON ? "true" : "false");
	strcat(buf, text);
	
	/* SystemStatus */
	memset(text, 0, sizeof(text));
	sprintf(text, ",\"SystemStatus\":{\"value\":%d}", g_system_status);
	strcat(buf, text);

	/* CarLock */
	memset(text, 0, sizeof(text));
	sprintf(text, ",\"CarLock\":{\"value\":%s}", g_car_lock ? "true" : "false");
	strcat(buf, text);

	/* Brake */
	memset(text, 0, sizeof(text));
	sprintf(text, ",\"Brake\":{\"value\":%s}", g_brake ? "true" : "false");
	strcat(buf, text);

	/* ParkingBrake */
	memset(text, 0, sizeof(text));
	sprintf(text, ",\"ParkingBrake\":{\"value\":%s}", g_parking_brake ? "true" : "false");
	strcat(buf, text);

	/* Distance */
	memset(text, 0, sizeof(text));
	sprintf(text, ",\"Distance\":{\"value\":%.1f}", (double)g_key_distance);
	strcat(buf, text);
	
	strcat(buf, "}}");
	return strlen(buf);
}

/*
************************************************************
*	��������:	OneNet_SendData
*	��������:	�� OneNET ƽ̨�ϱ��豸�������� (Post)
************************************************************
*/
void OneNet_SendData(void)
{
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};												
	char buf[256]; 
	char topic_buf[128];
	short body_len = 0;
	
    // 1. ��ȡ����õ� JSON ����
	memset(buf, 0, sizeof(buf));
	body_len = OneNet_FillBuf(buf);																	
	
	if(body_len)
	{
        // 2. ƴ����ģ�ͱ�׼�ϱ� Topic: $sys/{pid}/{device-name}/thing/property/post
		snprintf(topic_buf, sizeof(topic_buf), "$sys/%s/%s/thing/property/post", PROID, DEVICE_NAME);
		
        // 3. ���Ϊ MQTT PUBLISH ���Ĳ�����
        if(MQTT_PacketPublish(MQTT_PUBLISH_ID, topic_buf, buf, body_len, MQTT_QOS_LEVEL0, 0, 1, &mqttPacket) == 0)				
		{
			ESP8266_SendData(mqttPacket._data, mqttPacket._len);									
			MQTT_DeleteBuffer(&mqttPacket);															
		}
	}
}

/*
************************************************************
*	��������:	OneNET_Subscribe
*	��������:	���� OneNET ƽ̨���������� Topic
* ˵��:       ֻ�ж�����������⣬�豸�����յ��ƶ��·��Ŀ���ָ��
************************************************************
*/
void OneNET_Subscribe(void)
{
	MQTT_PACKET_STRUCTURE mqtt_packet = {NULL, 0, 0, 0};						
	char topic_buf[56]; 
    const char *topic = topic_buf;
    
    // ���ĵ� Topic: $sys/{pid}/{device-name}/thing/property/set
	snprintf(topic_buf, sizeof(topic_buf), "$sys/%s/%s/thing/property/set", PROID, DEVICE_NAME);
	
    if(MQTT_PacketSubscribe(MQTT_SUBSCRIBE_ID, MQTT_QOS_LEVEL0, &topic, 1, &mqtt_packet) == 0)
	{
		ESP8266_SendData(mqtt_packet._data, mqtt_packet._len);					
		MQTT_DeleteBuffer(&mqtt_packet);										
	}
}

/*
************************************************************
*	��������:	OneNet_RevPro
*	��������:	�����ƶ��·��� MQTT ���Ĳ�ִ��Ӳ������
* ��ڲ���:   cmd: ���յ���ԭʼ��������ָ��
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
	cJSON *lock_json, *brake_json, *park_json;
	char msg_id[32] = {0}; 
	char reply_topic[128];
	char reply_payload[128];
	MQTT_PACKET_STRUCTURE mqtt_reply = {NULL, 0, 0, 0};
	
    // 1. �жϱ�������
	type = MQTT_UnPacketRecv(cmd);
	switch(type)
	{
        // ƽ̨�·������� (ͨ���ǿ���ָ��)
		case MQTT_PKT_PUBLISH:
            // �����ȡ Payload (��Ч�غɣ��� JSON �ַ���)
			result = MQTT_UnPacketPublish(cmd, &cmdid_topic, &topic_len, &req_payload, &req_len, &qos, &pkt_id);
			if(result == 0)
			{
				req_payload[req_len] = '\0'; 
				UsartPrintf(USART_DEBUG, "\r\n[1] Recv Payload: %s\r\n", req_payload);
				
                // 2. ʹ�� cJSON �������յ��� JSON �ַ���
				raw_json = cJSON_Parse(req_payload);
				if(raw_json != NULL)
                {
                    // 3. ��ȡ msg_id (���ڸ�ƽ̨�ظ�ȷ�ϰ�ʱһһ��Ӧ)
                    id_json = cJSON_GetObjectItem(raw_json, "id");
                    if(id_json != NULL)
                    {
                        if(id_json->type == cJSON_String && id_json->valuestring != NULL)
                            strncpy(msg_id, id_json->valuestring, sizeof(msg_id) - 1);
                        else if(id_json->type == cJSON_Number || id_json->type == cJSON_False || id_json->type == cJSON_True)
                            snprintf(msg_id, sizeof(msg_id), "%d", id_json->valueint);
                    }
                    UsartPrintf(USART_DEBUG, "[2] Parsed msg_id: [%s]\r\n", msg_id);

                    // 4. �������Ʋ���������Ӳ��
                    params_json = cJSON_GetObjectItem(raw_json, "params");
                    if(params_json != NULL)
                    {
                        // ���� "Buzzer" ��
                        buzzer_json = cJSON_GetObjectItem(params_json, "Buzzer");
                        if(buzzer_json != NULL)
                        {
                            if(buzzer_json->type == cJSON_True || buzzer_json->type == 1 || buzzer_json->type == 2 || buzzer_json->valueint == 1) 
                                BUZZER_Set(BUZZER_ON);   
                            else 
                                BUZZER_Set(BUZZER_OFF);  
                            
                            UsartPrintf(USART_DEBUG, "[3] BUZZER Action Executed!\r\n");
                        }

                        /* CarLock */
                        lock_json = cJSON_GetObjectItem(params_json, "CarLock");
                        if (lock_json != NULL) {
                            if (lock_json->type == cJSON_True || lock_json->type == cJSON_False)
                                g_car_lock = (lock_json->type == cJSON_True) ? 1 : 0;
                            else
                                g_car_lock = (lock_json->valueint == 1) ? 1 : 0;
                            UsartPrintf(USART_DEBUG, "[3] CarLock = %d\r\n", g_car_lock);
                        }

                        /* Brake */
                        brake_json = cJSON_GetObjectItem(params_json, "Brake");
                        if (brake_json != NULL) {
                            if (brake_json->type == cJSON_True || brake_json->type == cJSON_False)
                                g_brake = (brake_json->type == cJSON_True) ? 1 : 0;
                            else
                                g_brake = (brake_json->valueint == 1) ? 1 : 0;
                            UsartPrintf(USART_DEBUG, "[3] Brake = %d\r\n", g_brake);
                        }

                        /* ParkingBrake */
                        park_json = cJSON_GetObjectItem(params_json, "ParkingBrake");
                        if (park_json != NULL) {
                            if (park_json->type == cJSON_True || park_json->type == cJSON_False)
                                g_parking_brake = (park_json->type == cJSON_True) ? 1 : 0;
                            else
                                g_parking_brake = (park_json->valueint == 1) ? 1 : 0;
                            UsartPrintf(USART_DEBUG, "[3] ParkingBrake = %d\r\n", g_parking_brake);
                        }
                    }
                    
                    // 5. ������ؼ�����ȡ��Ϻ������ͷ� cJSON �ڴ棬��ֹ���ڴ����
                    cJSON_Delete(raw_json);
                    
                    // 6. Ӳ��ִ����ϣ����ƶ˷��� ACK ȷ�ϰ� (set_reply)
                    if(strlen(msg_id) > 0)
                    {
                        snprintf(reply_topic, sizeof(reply_topic), "$sys/%s/%s/thing/property/set_reply", PROID, DEVICE_NAME);
                        snprintf(reply_payload, sizeof(reply_payload), "{\"id\":\"%s\",\"code\":200,\"msg\":\"success\"}", msg_id);
                        
                        UsartPrintf(USART_DEBUG, "[4] Ready to send ACK: %s\r\n", reply_payload);

                        // ��������ͻظ�
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
			
        // ���Ƿ��͵����ݣ�ƽ̨���ص�ȷ�ϰ�
		case MQTT_PKT_PUBACK:
			if(MQTT_UnPacketPublishAck(cmd) == 0) UsartPrintf(USART_DEBUG, "Tips: MQTT Publish Send OK\r\n");
		break;
        
        // ���Ƿ��͵Ķ�������ƽ̨���ص�ȷ�ϰ�
		case MQTT_PKT_SUBACK:
			if(MQTT_UnPacketSubscribe(cmd) == 0) UsartPrintf(USART_DEBUG, "Tips: MQTT Subscribe OK\r\n");
		break;
        
		default: result = -1; break;
	}
	
    // ��յײ���ջ�����
	ESP8266_Clear();
	if(result == -1) return;
	
    // �ͷ��� MQTT_UnPacketPublish �����ڲ�������ڴ�
	if(type == MQTT_PKT_CMD || type == MQTT_PKT_PUBLISH)
	{
		MQTT_FreeBuffer(cmdid_topic);
		MQTT_FreeBuffer(req_payload);
	}
}