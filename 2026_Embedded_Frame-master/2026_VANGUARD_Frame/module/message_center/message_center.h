/**
 * @file message_center.h
 * @author guatai (2508588132@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __MESSAGE_CENTER_H__
#define __MESSAGE_CENTER_H__

#include <stdint.h>

/***********************************
 描述
 用于将一段内存区域设置为指定的值。
 memset() 函数将指定的值 c 复制到 str 所指向的内存区域的前 n 个字节中，这可以用于将内存块清零或设置为特定值。
 在一些情况下，需要快速初始化大块内存为零或者特定值，memset() 可以提供高效的实现。
 在清空内存区域或者为内存区域赋值时，memset() 是一个常用的工具函数。
 声明
 void *memset(void *str, int c, size_t n)
 参数
 str -- 指向要填充的内存区域的指针。
 c -- 要设置的值，通常是一个无符号字符。
 n -- 要被设置为该值的字节数。
 返回值
 该值返回一个指向存储区 str 的指针。
 注意事项
 memset() 并不对指针 ptr 指向的内存区域做边界检查，因此使用时需要确保 ptr 指向的内存区域足够大，避免发生越界访问。
 memset() 的第二个参数 value 通常是一个 int 类型的值，但实际上只使用了该值的低8位。这意味着在范围 0 到 255 之外的其他值可能会产生未定义的行为。
 num 参数表示要设置的字节数，通常是通过 sizeof() 或其他手段计算得到的。
 ***********************************/

/***********************************
 描述
 void *memcpy(void *str1, const void *str2, size_t n) 从存储区 str2 复制 n 个字节到存储区 str1。
 声明
 void *memcpy(void *str1, const void *str2, size_t n)
 参数
 str1 -- 指向用于存储复制内容的目标数组，类型强制转换为 void* 指针。
 str2 -- 指向要复制的数据源，类型强制转换为 void* 指针。
 n -- 要被复制的字节数。
 返回值
 该函数返回一个指向目标存储区 str1 的指针。
 ***********************************/

#define MAX_TOPIC_NAME_LEN 32 // 最大的话题名长度,每个话题都有字符串来命名
#define MAX_TOPIC_COUNT 12    // 最多支持的话题数量
#define QUEUE_SIZE 1

typedef struct mqtt_sub
{
	/* 用数组模拟FIFO队列 */
	void *queue[QUEUE_SIZE];
	uint8_t data_len;
	uint8_t front_idx;
	uint8_t back_idx;
	uint8_t temp_size; // 当前队列长度

	/* 指向下一个订阅了相同的话题的订阅者的指针 */
	struct mqtt_sub *next_subs_queue; // 使得发布者可以通过链表访问所有订阅了相同话题的订阅者
} subscriber_t;

/**
 * @brief 发布者类型.每个发布者拥有发布者实例,并且可以通过链表访问所有订阅了自己发布的话题的订阅者
 *
 */
typedef struct mqtt_pub
{
	/* 话题名称 */
	char topic_name[MAX_TOPIC_NAME_LEN + 1]; // 1个字节用于存放字符串结束符 '\0'
	uint8_t data_len;                        // 该话题的数据长度
	/* 指向第一个订阅了该话题的订阅者,通过链表访问所有订阅者 */
	subscriber_t *first_subs;
	/* 指向下一个Publisher的指针 */
	struct mqtt_pub *next_topic_node;
	uint8_t pub_registered_flag; // 用于标记该发布者是否已经注册
} publisher_t;

/**
 * @brief 订阅name的话题消息
 *
 * @param name 话题名称
 * @param data_len 消息长度,通过sizeof()获取
 * @return subscriber_t* 返回订阅者实例
 */
subscriber_t *Subscriber_Register(char *name, uint8_t data_len);

/**
 * @brief 注册成为消息发布者
 *
 * @param name 发布者发布的话题名称(话题)
 * @return publisher_t* 返回发布者实例
 */
publisher_t *Publisher_Register(char *name, uint8_t data_len);

/**
 * @brief 获取消息
 *
 * @param sub 订阅者实例指针
 * @param data_ptr 数据指针,接收的消息将会放到此处
 * @return uint8_t 返回值为0说明没有新的消息(消息队列为空),为1说明获取到了新的消息
 */
uint8_t Subscriber_Get_Message(subscriber_t *sub, void *data_ptr);

/**
 * @brief 发布者给所有订阅了话题的订阅者推送消息
 *
 * @param pub 发布者实例指针
 * @param data_ptr 指向要发布的数据的指针
 * @return uint8_t 新消息成功推送给几个订阅者
 */
uint8_t Publisher_Push_Message(publisher_t *pub, void *data_ptr);

#endif /* __MESSAGE_CENTER_H__ */
