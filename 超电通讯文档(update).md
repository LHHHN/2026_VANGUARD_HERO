# 超级电容通讯文档

### 1.接收报文格式

标识符：0x06A	帧格式：DATA

帧类型：标准帧	DLC ：8字节

| 数据域  | 内容                                                         | 说明                                                         |
| ------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| DATA[0] | `enableDCDC`: 1<br />`systemRestart`: 1<br />`clearError`: 1<br />`resv0`: 5 | `enableDCDC`：使能DCDC<br />`systemRestart`：系统重启<br />`clearError`：手动清除可清除的错误 |
| DATA[1] | `refereePowerLimit`低八位                                    | 裁判限制功率，单位W                                          |
| DATA[2] | `refereePowerLimit`高八位                                    | 裁判限制功率，单位W                                          |
| DATA[3] | `refereeEnergyBuffer`低八位                                  | 裁判能量缓冲，单位J                                          |
| DATA[4] | `refereeEnergyBuffer`高八位                                  | 裁判能量缓冲，单位J                                          |
| DATA[5] | `resv1`                                                      |                                                              |
| DATA[6] | `resv1`                                                      |                                                              |
| DATA[7] | `resv2`                                                      | 填充位                                                       |



### 2.发送报文格式

标识符：0x05A	帧格式：DATA

帧类型：标准帧	DLC ：8字节

| 数据域  | 内容                  | 说明                                                         |
| ------- | --------------------- | ------------------------------------------------------------ |
| DATA[0] | `statusCode`          | 状态码，只需关注后两位：<br />0：无错误<br />1：错误，可通过自动恢复<br />2：错误，可通过发信息恢复<br />3：错误，不可恢复 |
| DATA[1] | `chassisPower`[00:07] | 底盘功率，单位W，float类型                                   |
| DATA[2] | `chassisPower`[08:15] | 底盘功率                                                     |
| DATA[3] | `chassisPower`[16:23] | 底盘功率                                                     |
| DATA[4] | `chassisPower`[24:31] | 底盘功率                                                     |
| DATA[5] | `chassisPowerLimit`   | 底盘最大可用功率（包括裁判系统），单位W                      |
| DATA[6] | `chassisPowerLimit`   | 底盘最大可用功率（包括裁判系统），单位W                      |
| DATA[7] | `capEnergy`           | 电容现有能量，映射到量程0-255                                |

发送频率：100Hz



> #### tips：
>
> 由于充放电保护，电容组能量低于10%时充电速度极慢，因此建议软件功控在能量用至10%之前打开，否则可能出现**<u>充不进电</u>**的情况