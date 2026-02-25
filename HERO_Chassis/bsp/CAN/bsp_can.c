/**
******************************************************************************
 * @file    bsp_can.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <stdlib.h>
#include <string.h>
#include "bsp_can.h"

#include "bsp_dwt.h"

/* can instance ptrs storage, used for recv callback */
// 在CAN产生接收中断会遍历数组,选出hfdcan和rxid与发生中断的实例相同的那个,调用其回调函数
// @todo: 后续为每个CAN总线单独添加一个can_instance指针数组,提高回调查找的性能
static CAN_instance_t *can_instances[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // 全局CAN实例索引,每次有新的模块注册会自增

/* ----------------two static function called by CAN_Register()-------------------- */

/**
 * @brief 添加过滤器以实现对特定id的报文的接收,会被CANRegister()调用
 *        给CAN添加过滤器后,BxCAN会根据接收到的报文的id进行消息过滤,符合规则的id会被填入FIFO触发中断
 *
 * @note h7的bxCAN有128个过滤器,这里将其配置为前14个过滤器给CAN1使用,中14个被CAN2使用，后14个被CAN3使用
 *       初始化时,奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
 *
 * @attention 你不需要完全理解这个函数的作用,因为它主要是用于初始化,在开发过程中不需要关心底层的实现
 *            享受开发的乐趣吧!如果你真的想知道这个函数在干什么,请联系作者或自己查阅资料(请直接查阅官方的reference manual)
 *
 * @param _instance can instance owned by specific module
 */
static void CAN_Add_Filter(CAN_instance_t *_instance)
{

#ifdef FDCAN	
	FDCAN_FilterTypeDef fdcan_filter_conf;
	static uint8_t can1_filter_idx = 0, can2_filter_idx = 14, can3_filter_idx = 28; // 0-41给can1用,42-83给can2用,84-127给can3用
	//检查是否超出过滤器设定数量上限
//	if(can1_filter_idx > hfdcan1.Init.StdFiltersNbr || can2_filter_idx > hfdcan2.Init.StdFiltersNbr || can3_filter_idx > hfdcan3.Init.StdFiltersNbr)
//	{
//		while(1)
//		{
//			//报错
//		}
//	}

	uint8_t *filter_idx_p;

	if(_instance->can_handle==&hfdcan1)
	{
		filter_idx_p=&can1_filter_idx;
	}
	else if(_instance->can_handle==&hfdcan2)
	{
		filter_idx_p=&can2_filter_idx;
	}
	else if(_instance->can_handle==&hfdcan3)
	{
		filter_idx_p=&can3_filter_idx;
	}
	else
	{
		while(1)
		{
			;//报错
		}
	}

	fdcan_filter_conf.IdType       = FDCAN_STANDARD_ID;                       //标准ID
	fdcan_filter_conf.FilterIndex  = (_instance->can_handle == &hfdcan1) ? (can1_filter_idx++) : ((_instance->can_handle == &hfdcan2) ? (can2_filter_idx++) : (can3_filter_idx++));                                  //滤波器索引                   
	fdcan_filter_conf.FilterType   = FDCAN_FILTER_MASK;
	// fdcan_filter_conf.FilterType   = FDCAN_FILTER_DUAL;                   //允许接收两个ID TODO: 后续可以优化使其能充分利用第二个ID位置
	fdcan_filter_conf.FilterConfig = (_instance->rx_id & 1) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;           //过滤器0关联到FIFO0  
	// fdcan_filter_conf.FilterID1    = _instance->rx_id;                               //32位ID接收ID1
	// fdcan_filter_conf.FilterID2    = _instance->rx_id;                    //接收ID2
	fdcan_filter_conf.FilterID1    = 0x000;                              
	fdcan_filter_conf.FilterID2    = 0x000; 
	if (HAL_FDCAN_ConfigFilter(_instance->can_handle, &fdcan_filter_conf) != HAL_OK)
	{
		Error_Handler( );
	}

#elif CAN
	CAN_FilterTypeDef can_filter_conf;
	static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用

	can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;                                                       // 使用id list模式,即只有将rxid添加到过滤器中才会接收到,其他报文会被过滤
	can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;                                                      // 使用16位id模式,即只有低16位有效
	can_filter_conf.FilterFIFOAssignment = (_instance->tx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;              // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
	can_filter_conf.SlaveStartFilterBank = 14;                                                                // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机)
	can_filter_conf.FilterIdLow = _instance->rx_id << 5;                                                      // 过滤器寄存器的低16位,因为使用STDID,所以只有低11位有效,高5位要填0
	can_filter_conf.FilterBank = _instance->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++); // 根据can_handle判断是CAN1还是CAN2,然后自增
	can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;                                                     // 启用过滤器

	HAL_CAN_ConfigFilter(_instance->can_handle, &can_filter_conf);
#endif	
}

/**
 * @brief 在第一个CAN实例初始化的时候会自动调用此函数,启动CAN服务
 *
 * @note 此函数会启动CAN1和CAN2,开启CAN1和CAN2的FIFO0 & FIFO1溢出通知
 *
 */
static void CAN_Service_Init( void )
{
#ifdef FDCAN
	uint32_t FDCAN_RXActiveITs = FDCAN_IT_RX_FIFO0_NEW_MESSAGE|FDCAN_IT_RX_FIFO0_FULL\
			|FDCAN_IT_RX_FIFO0_WATERMARK|FDCAN_IT_RX_FIFO0_MESSAGE_LOST \
			|FDCAN_IT_RX_FIFO1_NEW_MESSAGE| FDCAN_IT_RX_FIFO1_FULL\
			|FDCAN_IT_RX_FIFO1_WATERMARK|FDCAN_IT_RX_FIFO1_MESSAGE_LOST;

	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1,FDCAN_RX_FIFO1,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);//全局过滤器设置
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2,FDCAN_RX_FIFO1,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan3,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan3,FDCAN_RX_FIFO1,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO1, 1);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO1, 1);
	
#elif CAN
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
#endif
}

/* ----------------------- two extern callable function -----------------------*/

CAN_instance_t *CAN_Register(can_init_config_t *config)
{
	if (!idx)
	{
		CAN_Service_Init( ); // 第一次注册,先进行硬件初始化
	}
	if (idx >= CAN_MX_REGISTER_CNT) // 超过最大实例数
	{
		while (1);
	}
	for (size_t i = 0 ; i < idx ; i++)
	{ // 重复注册 | id重复
		if (can_instances[i]->rx_id == config->rx_id && can_instances[i]->can_handle == config->can_handle)
		{
			while (1);
		}
	}

	CAN_instance_t *instance = (CAN_instance_t *) malloc(sizeof(CAN_instance_t)); // 分配空间
	memset(instance, 0, sizeof(CAN_instance_t));                                                                     // 分配的空间未必是0,所以要先清空

#ifdef FDCAN	
	// 进行发送报文的配置
	instance->tx_header.Identifier          = config->tx_id; // 发送id
	instance->tx_header.IdType              = FDCAN_STANDARD_ID;															// 标准ID
	instance->tx_header.TxFrameType         = FDCAN_DATA_FRAME;														// 数据帧
	instance->tx_header.DataLength          = FDCAN_DLC_BYTES_8;														// 发送数据长度
	instance->tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;										        // 设置错误状态指示
	instance->tx_header.BitRateSwitch       = FDCAN_BRS_OFF;															// 不开启可变波特率
	instance->tx_header.FDFormat            = config->can_mode;															// 普通CAN格式
	instance->tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;										        // 用于发送事件FIFO控制, 不存储
	instance->tx_header.MessageMarker       = 0x00;
#elif CAN
	instance->txconf.StdId = config->tx_id; // 发送id
    instance->txconf.IDE = CAN_ID_STD;      // 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    instance->txconf.RTR = CAN_RTR_DATA;    // 发送数据帧
    instance->txconf.DLC = 0x08;            // 默认发送长度为8
#endif
	
	// 设置回调函数和接收发送id
	instance->can_handle          = config->can_handle;
	instance->tx_id               = config->tx_id; // 好像没用,可以删掉
	instance->rx_id               = config->rx_id;
	instance->can_module_callback = config->can_module_callback;
	instance->id                  = config->id;

	CAN_Add_Filter(instance);         // 添加CAN过滤器规则
	can_instances[idx++] = instance; // 将实例保存到can_instance中

	return instance; // 返回can实例指针
}

/* @todo 目前似乎封装过度,应该添加一个指向tx_buff的指针,tx_buff不应该由CAN instance保存 */
/* 如果让CANinstance保存txbuff,会增加一次复制的开销 */
uint8_t CAN_Transmit(CAN_instance_t *_instance, float timeout)
{
	static uint32_t busy_count;
	static volatile float wait_time __attribute__((unused)); // for cancel warning
	float dwt_start = DWT_GetTimeline_ms( );

#ifdef FDCAN	
	while (HAL_FDCAN_GetTxFifoFreeLevel(_instance->can_handle) == 0) // 等待邮箱空闲
#elif CAN
	while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0) // 等待邮箱空闲
#endif
	{
		if (DWT_GetTimeline_ms( ) - dwt_start > timeout) // 超时
		{
			busy_count++;
			if (busy_count > 300)
			{
				HAL_FDCAN_Stop(_instance->can_handle);
				HAL_FDCAN_AbortTxRequest(_instance->can_handle, FDCAN_TX_BUFFER0);
				HAL_FDCAN_AbortTxRequest(_instance->can_handle, FDCAN_TX_BUFFER1);
				HAL_FDCAN_AbortTxRequest(_instance->can_handle, FDCAN_TX_BUFFER2);
				HAL_FDCAN_Start(_instance->can_handle);
			}
			return 0;
		}
	}
	// wait_time = DWT_GetTimeline_ms() - dwt_start;
	//tx_mailbox会保存实际填入了这一帧消息的邮箱,但是知道是哪个邮箱发的似乎也没啥用
#ifdef FDCAN	
	if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->can_handle, &_instance->tx_header, _instance->tx_buff))
#elif CAN
	if (HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, _instance->tx_buff, &_instance->tx_mailbox))		
#endif	
    {
		// busy_count++;
		return 0;
	}
	return 1; // 发送成功
}

#ifdef FDCAN
/* 单次发送函数 */
uint8_t CAN_Transmit_Once(FDCAN_HandleTypeDef *can_handle, uint32_t StdId, uint8_t *tx_buff, float timeout)
{
	if (!idx)
	{
		CAN_Service_Init( ); // 判断是否进行过初始化,先进行硬件初始化
	}

	static CAN_instance_t tempTX_instance         = {0};
	tempTX_instance.tx_header.DataLength = FDCAN_DLC_BYTES_8;

	tempTX_instance.can_handle           = can_handle;
	tempTX_instance.tx_header.Identifier = StdId;
	memcpy(tempTX_instance.tx_buff, tx_buff, sizeof(tempTX_instance.tx_buff));

	CAN_Transmit(&tempTX_instance, timeout);
	return 1; // 发送成功
}
#endif

void CAN_Set_DLC(CAN_instance_t *_instance, uint8_t length)
{
	// 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
	if (length > 8 || length == 0) // 安全检查
		while (1)
			_instance->tx_header.DataLength = length;
}

/* -----------------------belows are callback definitions--------------------------*/

#ifdef FDCAN
/**
 * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
 *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
 *
 * @param _hfdcan
 * @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
 */
static void CANFIFOxCallback(FDCAN_HandleTypeDef *_hfdcan, uint32_t fifox)
{
	static FDCAN_RxHeaderTypeDef rx_config_header; // 同上
	static uint8_t can_rx_buff[64];
	while (HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
	{
		memset(can_rx_buff, 0, sizeof(can_rx_buff));
		HAL_FDCAN_GetRxMessage(_hfdcan, fifox, &rx_config_header, can_rx_buff); // 从FIFO中获取数据
		for (size_t i = 0 ; i < idx ; ++i)
		{ // 两者相等说明这是要找的实例
			if (_hfdcan == can_instances[i]->can_handle && rx_config_header.Identifier == can_instances[i]->rx_id)
			{
				if (can_instances[i]->can_module_callback != NULL) // 回调函数不为空就调用
				{
					can_instances[i]->rx_len = rx_config_header.DataLength;                   // 保存接收到的数据长度 TODO（GUATAI）:若接收出错可以考虑排查这里的处理是否有问题
					memcpy(can_instances[i]->rx_buff, can_rx_buff, can_instances[i]->rx_len); // 消息拷贝到对应实例
					can_instances[i]->can_module_callback(can_instances[i]);     // 触发回调进行数据解析和处理
				}
				return;
			}
		}
	}
}

/**
 * @brief 注意,STM32的两个CAN设备共享两个FIFO
 * 下面两个函数是HAL库中的回调函数,他们被HAL声明为__weak,这里对他们进行重载(重写)
 * 当FIFO0或FIFO1溢出时会调用这两个函数
 */
// 下面的函数会调用CANFIFOxCallback()来进一步处理来自特定CAN设备的消息

/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hfdcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	CANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hfdcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	CANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}

#elif CAN

/**
* @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
*        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
*
* @param _hcan
* @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
*/
static void CAN_FIFOx_Callback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
   static CAN_RxHeaderTypeDef rx_config_header; // 同上
   uint8_t can_rx_buff[8];
   while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
   {
       HAL_CAN_GetRxMessage(_hcan, fifox, &rx_config_header, can_rx_buff); // 从FIFO中获取数据
       for (size_t i = 0; i < idx; ++i)
       { // 两者相等说明这是要找的实例
           if (_hcan == can_instance[i]->can_handle && rx_config_header.StdId == can_instance[i]->rx_id)
           {
               if (can_instance[i]->can_module_callback != NULL) // 回调函数不为空就调用
               {
                   can_instance[i]->rx_len = rx_config_header.DLC;                      // 保存接收到的数据长度
                   memcpy(can_instance[i]->rx_buff, can_rx_buff, rx_config_header.DLC); // 消息拷贝到对应实例
                   can_instance[i]->can_module_callback(can_instance[i]);     // 触发回调进行数据解析和处理
               }
               return;
           }
       }
   }
}

/**
* @brief 注意,STM32的两个CAN设备共享两个FIFO
* 下面两个函数是HAL库中的回调函数,他们被HAL声明为__weak,这里对他们进行重载(重写)
* 当FIFO0或FIFO1溢出时会调用这两个函数
*/
// 下面的函数会调用CANFIFOxCallback()来进一步处理来自特定CAN设备的消息

/**
* @brief rx fifo callback. Once FIFO_0 is full,this func would be called
*
* @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
   CAN_FIFOx_Callback(hcan, CAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

/**
* @brief rx fifo callback. Once FIFO_1 is full,this func would be called
*
* @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
   CAN_FIFOx_Callback(hcan, CAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}

#endif