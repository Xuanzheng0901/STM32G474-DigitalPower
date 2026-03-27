#include "usart.h"
#include "string.h"

#include <unistd.h> // 必须包含此头文件

// 重写底层 _write 函数
int _write(int f, char *p, int l)
{
    // ptr 是缓冲区首地址，len 是本次要打印的字符串长度
    // 使用阻塞式发送，但因为是按块发送，效率远高于逐字节发送
    HAL_UART_Transmit(&huart3, (uint8_t *)p, l, HAL_MAX_DELAY);
    return l;
}
