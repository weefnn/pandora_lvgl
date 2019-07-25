/**
 * @file lv_port_indev_templ.c
 *
 */

 /*Copy this file as "lv_port_indev.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "button.h"
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
//static void button_init(void);
static bool button_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
static int8_t button_get_pressed_id(void);
static bool button_is_pressed(uint8_t id);

/**********************
 *  STATIC VARIABLES
 **********************/
lv_indev_t * indev_button;


/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void button_init(void)
{
    /* Here you will find example implementation of input devices supported by LittelvGL:
     *  - Touchpad
     *  - Mouse (with cursor support)
     *  - Keypad (supports GUI usage only with key)
     *  - Encoder (supports GUI usage only with: left, right, push)
     *  - Button (external buttons to press points on the screen)
     *
     *  The `..._read()` function are only examples.
     *  You should shape them according to your hardware
     */
    lv_indev_drv_t indev_drv;
    
    /*Register a button input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_BUTTON;
    indev_drv.read_cb = button_read;
    indev_button = lv_indev_drv_register(&indev_drv);

    /*Assign buttons to points on the screen*/
    static const lv_point_t btn_points[4] = {
            {10, 10},   /*KEY0  -> x:10; y:10*/
            {120, 10},  /*KEY1  -> x:40; y:100*/
            {180, 10},  /*KEY2  -> x:180; y:10*/
            {120, 190}, /*WK_UP -> x:120; y:190*/
    };
    lv_indev_set_button_points(indev_button, btn_points);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*------------------
 * Button
 * -----------------*/

/* Will be called by the library to read the button */
static bool button_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    static uint8_t last_btn = 0;

    /*Get the pressed button's ID*/
    int8_t btn_act = button_get_pressed_id();

    if(btn_act >= 0) {
        data->state = LV_INDEV_STATE_PR;
        last_btn = btn_act;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }

    /*Save the last pressed button's ID*/
    data->btn_id = last_btn;

    /*Return `false` because we are not buffering and no more data to read*/
    return false;
}

/*Get ID  (0, 1, 2 ..) of the pressed button*/
static int8_t button_get_pressed_id(void)
{
    uint8_t i;

    /*Check to buttons see which is being pressed (assume there are 2 buttons)*/
    for(i = 0; i < 4; i++) {
        /*Return the pressed button's ID*/
        if(button_is_pressed(i)) {
            return i;
        }
    }

    /*No button pressed*/
    return -1;
}

/*Test if `id` button is pressed or not*/
static bool button_is_pressed(uint8_t id)
{
    /*Your code comes here*/
    bool key_pressed = false;
    switch(id)
    {
        case 0:
            if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOD, KEY0_Pin))
                key_pressed = true;
            break;
        case 1:
            if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOD, KEY1_Pin))
                key_pressed = true;
            break;
        case 2:
            if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOD, KEY2_Pin))
                key_pressed = true;
            break;
        case 3:
            if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOC, WK_UP_Pin))
                key_pressed = true;
            break;
    }
    
    return key_pressed;
}

#else /* Enable this file at the top */

/* This dummy typedef exists purely to silence -Wpedantic. */
typedef int keep_pedantic_happy;
#endif
