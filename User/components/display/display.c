#include "display.h"
#include "display_internal.h"
#include "stm32g4xx_hal.h"
#include "dma.h"
#include "spi.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static xSemaphoreHandle spi_dma_sem;
static xSemaphoreHandle spi_mutex;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(hspi == &hspi1)
    {
        xSemaphoreGiveFromISR(spi_dma_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void display_switch_data_control(display_DC dc)
{
    HAL_GPIO_WritePin(SPI1_DC_GPIO_Port, SPI1_DC_Pin, (GPIO_PinState)dc);
}

void display_tx_cmd(uint8_t cmd, const uint8_t *param, size_t param_size)
{
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    display_switch_data_control(display_COMMAND);
    HAL_SPI_Transmit_DMA(&hspi1, &cmd, 1);
    xSemaphoreTake(spi_dma_sem, portMAX_DELAY);
    if(param != NULL && param_size > 0)
    {
        HAL_SPI_Transmit_DMA(&hspi1, param, param_size);
        xSemaphoreTake(spi_dma_sem, portMAX_DELAY);
    }
    xSemaphoreGive(spi_mutex);
}

void display_tx_data(uint8_t *data, size_t size)
{
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    display_switch_data_control(display_DATA);
    HAL_SPI_Transmit_DMA(&hspi1, data, size);
    xSemaphoreTake(spi_dma_sem, portMAX_DELAY);
    xSemaphoreGive(spi_mutex);
}

void display_init(void)
{
    spi_dma_sem = xSemaphoreCreateBinary();
    spi_mutex = xSemaphoreCreateMutex();

    xSemaphoreTake(spi_dma_sem, 0);

#if DISPLAY_USE == DISPLAY_SH1107
    sh1107_init();
#elif DISPLAY_USE == DISPLAY_SSD1306
    ssd1306_init();
#endif

    LOGI("OLED", "初始化完成");
}
