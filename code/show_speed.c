#include "show_speed.h"
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
#include "car_headfile.h"
seekfree_assistant_oscilloscope_struct oscilloscope_data;
void show_speed_init(void){
    // 设置逐飞助手使用DEBUG串口进行收发
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);

    // 初始化逐飞助手示波器的结构体


       oscilloscope_data.data[0] = 0;
       oscilloscope_data.data[1] = 0;
       // 设置为2个通道，通道数量最大为8个
       oscilloscope_data.channel_num = 2;
}
void show_speed_by_uart(void){

    oscilloscope_data.data[0] = car.left_motor.current_speed;
    oscilloscope_data.data[1] = car.right_motor.current_speed;
    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
}

void controller_by_uart(void){
    // 滴答客解析接收到的数据
    seekfree_assistant_data_analysis();

    // 遍历
    for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
    {
        // 更新标志位
        if(seekfree_assistant_parameter_update_flag[i])
        {
                seekfree_assistant_parameter_update_flag[i] = 0;

                // 通过DEBBUG串口发送信息
                printf("receive data channel : %d ", i);
                printf("data : %f ", seekfree_assistant_parameter[i]);
                printf("\r\n");
                switch (i)
                {
                case 0: // 通道0 控制电机速度
                    smart_car.pid_configs[smart_car.current_pid_scene].base_speed = (int16_t)(seekfree_assistant_parameter[i]);
                    /* code */
                    break;
                case 1: // 通道1 控制电机kp
                    smart_car.pid_configs[smart_car.current_pid_scene].speed.kp = seekfree_assistant_parameter[i];
                    /* code */
                    break;
                case 2: // 通道2 控制电机ki
                    smart_car.pid_configs[smart_car.current_pid_scene].speed.ki = seekfree_assistant_parameter[i];
                    /* code */
                    break;
                case 3: // 通道3 控制电机kd
                    smart_car.pid_configs[smart_car.current_pid_scene].speed.kd = seekfree_assistant_parameter[i];
                    /* code */
                    break;
                case 4: // 通道4 控制舵机kp
                    smart_car.pid_configs[smart_car.current_pid_scene].direction.kp = seekfree_assistant_parameter[i];
                    /* code */
                    break;
                case 5: // 通道5 控制舵机kd
                    smart_car.pid_configs[smart_car.current_pid_scene].direction.kd = seekfree_assistant_parameter[i];
                    /* code */
                    break;
                default:
                    break;
                }
        }
    }
}
