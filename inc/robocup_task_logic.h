/**
 * @file  robocup_task_logic.h
 * @brief RoboCup 搬运小车高层任务逻辑头文件
 * @author 2026 开飞机的娃子 (1211959902@qq.com)
 * @license MIT
 */
#ifndef ROBOCUP_TASK_LOGIC_H
#define ROBOCUP_TASK_LOGIC_H

#include <stdint.h>

/* ==================== 物料参数配置 ==================== */
#define POS_RED    0
#define POS_RED1   1
#define POS_RED2   2
#define POS_BLUE1  3
#define POS_BLUE2  4
#define POS_NUM    5

#define EXPECT_RED_PER_ROUND   2
#define EXPECT_BLUE_PER_ROUND  2
#define EXPECT_TOTAL_PER_ROUND 4

#define RECHECK_TIMEOUT_MS      3000
#define PUT_RECHECK_TIMEOUT_MS  2000

/* 形状枚举 */
#define SHAPE_CYLINDER   1
#define SHAPE_CUBE       2
#define SHAPE_UNKNOWN    3

/* ==================== 奖杯参数配置 ==================== */
#define POS_6        0
#define POS_7        1
#define POS_8        2
#define TROPHY_POS_NUM   3

#define MIN_TOTAL_SCORE   1.20f
#define MIN_MAX_CONF      0.45f
#define SINGLE_RATIO_HIGH  0.50f
#define SINGLE_RATIO_DIFF  0.15f

/* ==================== 对外暴露的 API ==================== */
/**
 * @brief 抓取物料（带超时与推理兜底）
 * @param materialNum 第几个物料 (1~4为红蓝，5特判为黄锥)
 */
void grab1(int materialNum);

/**
 * @brief 放置物料（圆柱优先调度）
 * @param PutmaterialNum 1=红，2=蓝
 */
void put1(int PutmaterialNum);

/**
 * @brief 抓取奖杯并采样
 * @param materialNum1 第几个奖杯 (0~2)
 */
void newgrab1(int materialNum1);

/**
 * @brief 放置奖杯
 * @param rank 名次 (0=冠, 1=亚, 2=季)
 */
void newput1(int rank);

#endif // ROBOCUP_TASK_LOGIC_H