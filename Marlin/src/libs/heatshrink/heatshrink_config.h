/** translatione by yx */
/**
 * libs/heatshrink/heatshrink_config.h
 */
#pragma once

// Should functionality assuming dynamic allocation be used?//是否应使用假设动态分配的功能？
#ifndef HEATSHRINK_DYNAMIC_ALLOC
  //#define HEATSHRINK_DYNAMIC_ALLOC 1//#定义热收缩\u动态\u ALLOC 1
#endif

#if HEATSHRINK_DYNAMIC_ALLOC
  // Optional replacement of malloc/free//malloc/free的可选更换
  #define HEATSHRINK_MALLOC(SZ) malloc(SZ)
  #define HEATSHRINK_FREE(P, SZ) free(P)
#else
  // Required parameters for static configuration//静态配置所需的参数
  #define HEATSHRINK_STATIC_INPUT_BUFFER_SIZE 32
  #define HEATSHRINK_STATIC_WINDOW_BITS 8
  #define HEATSHRINK_STATIC_LOOKAHEAD_BITS 4
#endif

// Turn on logging for debugging//打开日志以进行调试
#define HEATSHRINK_DEBUGGING_LOGS 0

// Use indexing for faster compression. (This requires additional space.)//使用索引可以加快压缩速度。（这需要额外的空间。）
#define HEATSHRINK_USE_INDEX 1
