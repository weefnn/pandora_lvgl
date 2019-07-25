/**
 * @file lv_port_disp_templ.c
 *
 */

 /*Copy this file as "lv_port_disp.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "tft.h"
#include "lcd.h"
#include "main.h"
/*********************
 *      DEFINES
 *********************/
 
/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);

/**********************
 *  STATIC VARIABLES
 **********************/
static lv_disp_drv_t display_drv;                         /*Descriptor of a display driver*/
#if USB_SPI_DMA
static lv_disp_drv_t * disp_p;
#endif
/**********************
 *      MACROS
 **********************/
extern SPI_HandleTypeDef hspi3;
extern DMA_HandleTypeDef hdma_spi3_tx;
/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void tft_init(void)
{
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    disp_init();

    /*-----------------------------
     * Create a buffer for drawing
     *----------------------------*/
    /* Example for 1) */
//    static lv_disp_buf_t disp_buf_1;
//    static lv_color_t buf1_1[LV_HOR_RES_MAX * 40];                      /*A buffer for 10 rows*/
//    lv_disp_buf_init(&disp_buf_1, buf1_1, NULL, LV_HOR_RES_MAX * 40);   /*Initialize the display buffer*/

    /* Example for 2) */
    static lv_disp_buf_t disp_buf_2;
    static lv_color_t buf2_1[LV_HOR_RES_MAX * 40];                        /*A buffer for 10 rows*/
    static lv_color_t buf2_2[LV_HOR_RES_MAX * 40];                        /*An other buffer for 10 rows*/
    lv_disp_buf_init(&disp_buf_2, buf2_1, buf2_2, LV_HOR_RES_MAX * 40);   /*Initialize the display buffer*/

    /*-----------------------------------
     * Register the display in LittlevGL
     *----------------------------------*/
    lv_disp_drv_init(&display_drv);                    /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    display_drv.hor_res = 240;
    display_drv.ver_res = 240;

    /*Used to copy the buffer's content to the display*/
    display_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    display_drv.buffer = &disp_buf_2;

    /*Finally register the driver*/
    lv_disp_drv_register(&display_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/* Initialize your display and the required peripherals. */
static void disp_init(void)
{
    /*You code here*/
}

/* Flush the content of the internal buffer the specific area on the display
 * You can use DMA or any hardware acceleration to do this operation in the background but
 * 'lv_disp_flush_ready()' has to be called when finished. */
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    /*SPI transmit data without DMA*/
#if USB_SPI_DMA
    /*SPI transmit data with DMA*/
    LCD_Send_Data_DMA(area->x1, area->y1, area->x2, area->y2, (uint8_t *)color_p);
    disp_p = disp_drv;
#else
    LCD_Send_Data(area->x1, area->y1, area->x2, area->y2, (uint8_t *)color_p);
    lv_disp_flush_ready(disp_drv);
#endif
}

#if USB_SPI_DMA
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    lv_disp_flush_ready(disp_p);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    HAL_Delay(500);
}
#endif

#else /* Enable this file at the top */

/* This dummy typedef exists purely to silence -Wpedantic. */
typedef int keep_pedantic_happy;
#endif
