/**
 * @file lv_port_indev_templ.c
 *
 */

 /*Copy this file as "lv_port_indev.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "keypad.h"
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

static bool keypad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
static uint32_t keypad_get_key(void);

/**********************
 *  STATIC VARIABLES
 **********************/
lv_indev_t * indev_keypad;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void keypad_init(void)
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

    /*------------------
     * Keypad
     * -----------------*/
    /*Register a keypad input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_KEYPAD;
    indev_drv.read_cb = keypad_read;
    indev_keypad = lv_indev_drv_register(&indev_drv);

    /* Later you should create group(s) with `lv_group_t * group = lv_group_create()`,
     * add objects to the group with `lv_group_add_obj(group, obj)`
     * and assign this input device to group to navigate in it:
     * `lv_indev_set_group(indev_keypad, group);` */
//    lv_group_t * group = lv_group_create();
//    //lv_group_add_obj(group, obj);
//    lv_indev_set_group(indev_keypad, group);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*------------------
 * Keypad
 * -----------------*/
/* Will be called by the library to read the mouse */
static bool keypad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    static uint32_t last_key = 0;

    /*Get the current x and y coordinates*/
    //mouse_get_xy(&data->point.x, &data->point.y);

    /*Get whether the a key is pressed and save the pressed key*/
    uint32_t act_key = keypad_get_key();
    if(act_key != 0) {
        data->state = LV_INDEV_STATE_PR;

        /*Translate the keys to LittlevGL control characters according to your key definitions*/
        switch(act_key) {
        case 1:
            act_key = LV_KEY_NEXT;
            break;
        case 2:
            act_key = LV_KEY_PREV;
            break;
        case 3:
            act_key = LV_KEY_LEFT;
            break;
        case 4:
            act_key = LV_KEY_RIGHT;
            break;
        case 5:
            act_key = LV_KEY_ENTER;
            break;
        }

        last_key = act_key;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }

    data->key = last_key;

    /*Return `false` because we are not buffering and no more data to read*/
    return false;
}

/*Get the currently being pressed key.  0 if no key is pressed*/
static uint32_t keypad_get_key(void)
{
    /*Your code comes here*/
    uint32_t key_num = 0;

    if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOD, KEY0_Pin))
        key_num = 1;
    else if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOD, KEY1_Pin))
        key_num = 2;
    else if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOD, KEY2_Pin))
        key_num = 4;
    else if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOC, WK_UP_Pin))
        key_num = 5;

    return key_num;
}

#else /* Enable this file at the top */

/* This dummy typedef exists purely to silence -Wpedantic. */
typedef int keep_pedantic_happy;
#endif
