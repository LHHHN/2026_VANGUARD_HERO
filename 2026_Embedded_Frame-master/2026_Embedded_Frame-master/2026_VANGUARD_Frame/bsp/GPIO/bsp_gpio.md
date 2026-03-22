# bso_gpio

# 请注意GPIO设备可以作为io接口,也可以处理外部中断.后续打算能够直接在此配置GPIO，无需在CubeMX中配置，以适应某些需要转换GPIO输入输出的场景

## 使用说明

## 使用示例

```c
//在app层只需要设置前三个

GPIO_init_config_t gpio_init = {
    .exti_mode = GPIO_EXTI_MODE_FALLING, // 注意和CUBEMX的配置一致
    .GPIO_Pin = GPIO_PIN_6, // GPIO引脚
    .GPIOx = GPIOG, // GPIO外设
    .gpio_model_callback = NULL, // EXTI回调函数
},

GPIO_instance_t *test_example = GPIO_Register(&gpio_init);
GPIO_Set(test_example);
// GPIO_xxx(test_exmaple, ...);
```
