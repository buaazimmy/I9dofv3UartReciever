#include <stdio.h>
#include "serial_port.h"
using namespace std;
//---------------AHRS 数据接口部分------------------------------------------------

//uart reicer flag
#define b_uart_head  0x80  //收到A5 头 标志位
#define b_rx_over    0x40  //收到完整的帧标志
// USART Receiver buffer
#define RX_BUFFER_SIZE 100 //接收缓冲区字节数

void Decode_frame(unsigned char data);
volatile unsigned char rx_buffer[RX_BUFFER_SIZE]; //接收数据缓冲区
volatile unsigned char rx_wr_index; //缓冲写指针
volatile unsigned char RC_Flag;  //接收状态标志字节

//解算后的角度值
float   yaw,  //偏航角
        pitch,//俯仰
        roll, //滚转
        alt,  //高度
        tempr,//温度
        press;//气压
//ADC值
int ax, ay, az;//加速度计
int gx, gy, gz;//陀螺仪
int hx, hy, hz;//磁力计

//在接收完一帧IMU姿态报告后，调用这个子程序来取出姿态数据
void UART2_Get_IMU(void)
{
    int temp;

    temp = 0;
    temp = rx_buffer[2];
    temp <<= 8;
    temp |= rx_buffer[3];
    if (temp & 0x8000) {
        temp = 0 - (temp & 0x7fff);
    } else temp = (temp & 0x7fff);
    yaw = (float)temp / 10.0f; //偏航角

    temp = 0;
    temp = rx_buffer[4];
    temp <<= 8;
    temp |= rx_buffer[5];
    if (temp & 0x8000) {
        temp = 0 - (temp & 0x7fff);
    } else temp = (temp & 0x7fff);
    pitch = (float)temp / 10.0f; //俯仰

    temp = 0;
    temp = rx_buffer[6];
    temp <<= 8;
    temp |= rx_buffer[7];
    if (temp & 0x8000) {
        temp = 0 - (temp & 0x7fff);
    } else temp = (temp & 0x7fff);
    roll = (float)temp / 10.0f; //滚转

    temp = 0;
    temp = rx_buffer[8];
    temp <<= 8;
    temp |= rx_buffer[9];
    if (temp & 0x8000) {
        temp = 0 - (temp & 0x7fff);
    } else temp = (temp & 0x7fff);
    alt = (float)temp / 10.0f; //高度

    temp = 0;
    temp = rx_buffer[10];
    temp <<= 8;
    temp |= rx_buffer[11];
    if (temp & 0x8000) {
        temp = 0 - (temp & 0x7fff);
    } else temp = (temp & 0x7fff);
    tempr = (float)temp / 10.0f; //温度

    temp = 0;
    temp = rx_buffer[12];
    temp <<= 8;
    temp |= rx_buffer[13];
    if (temp & 0x8000) {
        temp = 0 - (temp & 0x7fff);
    } else temp = (temp & 0x7fff);
    press = (float)temp * 10.0f; //气压

}

//在接收一帧ReportMotion 后调用这个子程序来取出ADC数据
void UART2_Get_Motion(void)
{
    int temp;

    temp = 0;
    temp = rx_buffer[2];
    temp <<= 8;
    temp |= rx_buffer[3];
    if (temp & 0x8000) {
        temp = 0 - (temp & 0x7fff);
    } else temp = (temp & 0x7fff);
    ax = temp; //加速度计 X轴的ADC值

    temp = 0;
    temp = rx_buffer[4];
    temp <<= 8;
    temp |= rx_buffer[5];
    if (temp & 0x8000) {
        temp = 0 - (temp & 0x7fff);
    } else temp = (temp & 0x7fff);
    ay = temp; //加速度计 Y轴的ADC值

    temp = 0;
    temp = rx_buffer[6];
    temp <<= 8;
    temp |= rx_buffer[7];
    if (temp & 0x8000) {
        temp = 0 - (temp & 0x7fff);
    } else temp = (temp & 0x7fff);
    az = temp; //加速度计 Z轴的ADC值

    temp = 0;
    temp = rx_buffer[8];
    temp <<= 8;
    temp |= rx_buffer[9];
    if (temp & 0x8000) {
        temp = 0 - (temp & 0x7fff);
    } else temp = (temp & 0x7fff);
    gx = temp; //陀螺仪 X轴的ADC值

    temp = 0;
    temp = rx_buffer[10];
    temp <<= 8;
    temp |= rx_buffer[11];
    if (temp & 0x8000) {
        temp = 0 - (temp & 0x7fff);
    } else temp = (temp & 0x7fff);
    gy = temp; //陀螺仪 Y轴的ADC值

    temp = 0;
    temp = rx_buffer[12];
    temp <<= 8;
    temp |= rx_buffer[13];
    if (temp & 0x8000) {
        temp = 0 - (temp & 0x7fff);
    } else temp = (temp & 0x7fff);
    gz = temp; //陀螺仪 Z轴的ADC值

    temp = 0;
    temp = rx_buffer[14];
    temp <<= 8;
    temp |= rx_buffer[15];
    if (temp & 0x8000) {
        temp = 0 - (temp & 0x7fff);
    } else temp = (temp & 0x7fff);
    hx = temp; //磁力计 X轴的ADC值

    temp = 0;
    temp = rx_buffer[16];
    temp <<= 8;
    temp |= rx_buffer[17];
    if (temp & 0x8000) {
        temp = 0 - (temp & 0x7fff);
    } else temp = (temp & 0x7fff);
    hy = temp; //磁力计 Y轴的ADC值

    temp = 0;
    temp = rx_buffer[18];
    temp <<= 8;
    temp |= rx_buffer[19];
    if (temp & 0x8000) {
        temp = 0 - (temp & 0x7fff);
    } else temp = (temp & 0x7fff);
    hz = temp; //磁力计 Z轴的ADC值
}

//--校验当前接收到的一帧数据是否 与帧校验字节一致
unsigned char Sum_check(void)
{
    unsigned char i;
    unsigned int checksum = 0;
    for (i = 0; i < rx_buffer[0] - 2; i++)
        checksum += rx_buffer[i];
    if ((checksum % 256) == rx_buffer[rx_buffer[0] - 2])
        return (0x01); //Checksum successful
    else
        return (0x00); //Checksum error
}


void OnAhrs_Data(unsigned char data) {

    if (data == 0xa5)
    {
        RC_Flag |= b_uart_head; //如果接收到A5 置位帧头标专位
        rx_buffer[rx_wr_index++] = data; //保存这个字节.
    }
    else if (data == 0x5a)
    {
        if (RC_Flag & b_uart_head) //如果上一个字节是A5 那么认定 这个是帧起始字节
        {   rx_wr_index = 0; //重置 缓冲区指针
            RC_Flag &= ~b_rx_over; //这个帧才刚刚开始收
        }
        else //上一个字节不是A5
            rx_buffer[rx_wr_index++] = data;
        RC_Flag &= ~b_uart_head; //清帧头标志
    }
    else
    {   rx_buffer[rx_wr_index++] = data;
        RC_Flag &= ~b_uart_head;
        if (rx_wr_index == rx_buffer[0]) //收够了字节数.
        {
            RC_Flag |= b_rx_over; //置位 接收完整的一帧数据

        }
    }

    if (rx_wr_index == RX_BUFFER_SIZE) //防止缓冲区溢出
        rx_wr_index--;


}
void printf_binary(unsigned int n)
{
    int i;
    for (i = 7; i >= 0; i--)
    {
        printf("%d", ((n & ((1 << (i + 1)) - 1)) >> i) ? 1 : 0);
        if (i % 8 == 0)
            printf(" ");
    }
}

int main()
{
    Serial_Port serial;

    uint8_t ch;

    serial.start();

    while (1) {
        serial.read_port(ch);
        OnAhrs_Data(ch);

        if (RC_Flag & b_rx_over) { //已经接收完一帧?
            RC_Flag &= ~b_rx_over; //清标志先
            if (Sum_check()) {//校验通过
                if (rx_buffer[1] == 0xA1) { //UART2_ReportIMU 的数据
                    UART2_Get_IMU();    //取数据
                    printf("YPR:%7.2f%7.2f%7.2f  Alt:%7.2f  Press: %9.1f  Tempr: %5.2f\t", yaw, pitch, roll, alt, press, tempr);
                }
                if (rx_buffer[1] == 0xA2) { //UART2_ReportMotion 的数据
                    UART2_Get_Motion();  //取数据
                    printf("acc:%6.2f %6.2f %6.2f\tgyro:%6.2f %6.2f %6.2f \tmagnet:%5.2f %5.2f %5.2f \n",
                           (float)ax / 13107.0, (float)ay / 13107.0, (float)az / 13107.0,
                           (float)gx / 78.6, (float)gy / 78.6, (float)gz / 78.6,
                           (float)hx * 0.73, (float)hy * 0.73, (float)hz * 0.73);
//                    printf("acc:%6d%6d%6d\tgyro:%6d%6d%6d\tmagnet:%5d%5d%5d\n", ax, ay, az, gx, gy, gz, hx, hy, hz);
                }
            }
        }
    }
    serial.stop();



    return 0;
}
