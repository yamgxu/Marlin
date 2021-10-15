/** translatione by yx */
/**
 * @file    fontutils.h
 * @brief   help functions for font and char
 * @author  Yunhui Fu (yhfudev@gmail.com)
 * @version 1.0
 * @date    2016-08-19
 * @copyright GPL/BSD
 */
#pragma once

#include <stdlib.h>
#include <stddef.h> // wchar_t//wchar\t
#include <stdint.h> // uint32_t//uint32\u t

#include "../HAL/shared/Marduino.h"
#include "../core/macros.h"

// read a byte from ROM or RAM//从ROM或RAM中读取字节
typedef uint8_t (*read_byte_cb_t)(uint8_t * str);

uint8_t read_byte_ram(uint8_t * str);
uint8_t read_byte_rom(uint8_t * str);

// there's overflow of the wchar_t due to the 2-byte size in Arduino//由于Arduino中的2字节大小，wchar\u t溢出
// sizeof(wchar_t)=2; sizeof(size_t)=2; sizeof(uint32_t)=4;//sizeof（wchar_t）=2；sizeof（size_t）=2；sizeof（uint32_t）=4；
// sizeof(int)=2; sizeof(long)=4; sizeof(unsigned)=2;//sizeof（int）=2；sizeof（long）=4；sizeof（无符号）=2；
//#undef wchar_t//#未定义wchar\t
#define wchar_t uint32_t
//typedef uint32_t wchar_t;//类型定义uint32\u t wchar\t；

typedef uint16_t pixel_len_t;
#define PIXEL_LEN_NOLIMIT ((pixel_len_t)(-1))

/* Perform binary search */
typedef int (* pf_bsearch_cb_comp_t)(void *userdata, size_t idx, void * data_pin); /*"data_list[idx] - *data_pin"*/
int pf_bsearch_r(void *userdata, size_t num_data, pf_bsearch_cb_comp_t cb_comp, void *data_pinpoint, size_t *ret_idx);

/* Get the character, decoding multibyte UTF8 characters and returning a pointer to the start of the next UTF8 character */
uint8_t* get_utf8_value_cb(uint8_t *pstart, read_byte_cb_t cb_read_byte, wchar_t *pval);

/* Returns length of string in CHARACTERS, NOT BYTES */
uint8_t utf8_strlen(const char *pstart);
uint8_t utf8_strlen_P(PGM_P pstart);

/* Returns start byte position of desired char number */
uint8_t utf8_byte_pos_by_char_num(const char *pstart, const uint8_t charnum);
uint8_t utf8_byte_pos_by_char_num_P(PGM_P pstart, const uint8_t charnum);
