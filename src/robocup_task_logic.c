/**
 * @file  robocup_task_logic.c
 * @brief 搬运/放置任务高层状态机与推理算法
 * @author 2026 开飞机的娃子 (1211959902@qq.com)
 * @license MIT
 */
#include "robocup_task_logic.h"
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"

/* ==================== 外部底层接口（需移植对接） ==================== */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern uint8_t WuLiaoData;
extern void SetMaixCamMode(uint8_t mode);
extern uint8_t Car_Adjust_All(uint8_t type);
extern void HuaTai_Z(int32_t val);
extern void HuaTai_X(int32_t val);
extern void change_direction(int dir);
extern void move_distance_namara_car(int16_t x, int16_t y, int16_t rot, uint16_t speed, uint16_t time);

/* 舵机常量 */
extern uint16_t mid, zhua1, zhua2, zhua3, zhua4;
extern uint16_t COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_BLACK, COLOR_WHITE;
extern uint16_t COLOR_RED_SERVO, COLOR_BLACK_SERVO, COLOR_WHITE_SERVO;

/* 视觉模式宏 */
extern uint8_t MAIXCAM_MODE_WULIAO;
extern uint8_t cylinder;
extern uint8_t Annulus, Annulus2, Annulus3, Annulus4, Annulus6; // 补全Annulus6

/* ==================== 静态全局变量 ==================== */
static uint8_t pos_used[POS_NUM]    = {0};
static uint8_t pos_content[POS_NUM] = {0};
static uint8_t pos_shape[POS_NUM]   = {0};

static const uint16_t pos_servo[POS_NUM] = {
    COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_BLACK, COLOR_WHITE
};

static const uint8_t red_order[]  = { POS_RED1, POS_RED2 };
static const uint8_t blue_order[] = { POS_BLUE1, POS_BLUE2 };
static const uint8_t fail_order[] = { POS_RED, POS_RED2, POS_BLUE2, POS_RED1, POS_BLUE1 };
static const uint8_t all_pos[]    = { POS_RED, POS_RED1, POS_RED2, POS_BLUE1, POS_BLUE2 };

static uint8_t round_red     = 0;
static uint8_t round_blue    = 0;
static uint8_t round_unknown = 0;
static uint8_t round_total   = 0;

/* 奖杯相关 */
static const uint16_t trophy_servo[TROPHY_POS_NUM] = {
    COLOR_RED_SERVO, COLOR_BLACK_SERVO, COLOR_WHITE_SERVO
};
static int g_vote6[3] = {0}, g_vote7[3] = {0}, g_vote8[3] = {0}, g_total[3] = {0};
static uint8_t g_phys_pos[3] = { POS_6, POS_7, POS_8 };
static uint8_t g_final_val[3] = {0, 0, 0};
static uint8_t g_target_rank[3] = {0xFF, 0xFF, 0xFF};
static uint8_t g_result_valid = 0;

/* ==================== 内部辅助函数 ==================== */
static uint8_t get_shape_from_wuliao(uint8_t dat) {
    if (dat == 1 || dat == 3) return SHAPE_CYLINDER;
    if (dat == 2 || dat == 4) return SHAPE_CUBE;
    return SHAPE_UNKNOWN;
}

static uint8_t find_empty_in(const uint8_t *order, uint8_t len) {
    for (uint8_t i = 0; i < len; i++)
        if (!pos_used[order[i]]) return order[i];
    return 0xFF;
}

static uint8_t find_empty_global(void) { return find_empty_in(all_pos, POS_NUM); }

static uint8_t find_blue_cyl_pos(void) {
    if (!pos_used[POS_BLUE2]) return POS_BLUE2;
    if (!pos_used[POS_BLUE1]) return POS_BLUE1;
    return find_empty_global();
}

static uint8_t find_blue_cube_pos(void) {
    if (!pos_used[POS_BLUE1]) return POS_BLUE1;
    if (!pos_used[POS_BLUE2]) return POS_BLUE2;
    return find_empty_global();
}

static void place_at(uint8_t pos, uint8_t content, uint8_t shape) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pos_servo[pos]);
    pos_used[pos]    = 1;
    pos_content[pos] = content;
    pos_shape[pos]   = shape;
    printf("放置位置%d, 颜色=%s, 形状=%s\r\n", pos,
           content == 1 ? "红" : content == 2 ? "蓝" : "未知",
           shape == 1 ? "圆柱" : shape == 2 ? "方块" : "未知");
}

static uint8_t infer_color(void) {
    uint8_t red_left   = EXPECT_RED_PER_ROUND  - round_red;
    uint8_t blue_left  = EXPECT_BLUE_PER_ROUND - round_blue;
    uint8_t total_left = EXPECT_TOTAL_PER_ROUND - round_total;

    if (red_left == 0 && blue_left > 0)  return 2;
    if (blue_left == 0 && red_left > 0)  return 1;
    if (total_left == 1) {
        if (red_left == 1 && blue_left == 0) return 1;
        if (blue_left == 1 && red_left == 0) return 2;
    }
    return 3;
}

static void round_check_and_reset(void) {
    printf("== 本轮对账: 红=%d, 蓝=%d, 未知=%d ==\r\n", round_red, round_blue, round_unknown);
    round_red = round_blue = round_unknown = round_total = 0;
}

static uint8_t grab_recheck(void) {
    uint32_t start_tick = HAL_GetTick();
    SetMaixCamMode(MAIXCAM_MODE_WULIAO);
    WuLiaoData = 0;
    osDelay(300);

    while ((HAL_GetTick() - start_tick) < RECHECK_TIMEOUT_MS) {
        osDelay(50);
        if      (WuLiaoData == 1 || WuLiaoData == 2) return 1;
        else if (WuLiaoData == 3 || WuLiaoData == 4) return 2;
        else if (WuLiaoData == 5)                    return 3;
    }
    return 0;
}

static uint8_t put_recheck_pos(uint8_t pos) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pos_servo[pos]);
    osDelay(800);
    WuLiaoData = 0;
    osDelay(300);
    if (WuLiaoData == 1 || WuLiaoData == 2) return 1;
    if (WuLiaoData == 3 || WuLiaoData == 4) return 2;
    return 0;
}

/* ==================== 物料主逻辑 ==================== */
void grab1(int materialNum) {
    static uint8_t grab_count = 0;
    
    // 第5个物料特殊通道（黄锥）
    if (materialNum == 5) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, mid);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1250);
        osDelay(500);
        HuaTai_Z(-700); osDelay(300);
        Car_Adjust_All(cylinder); osDelay(500);
        change_direction(0); osDelay(500);
        change_direction(0); osDelay(500);
        move_distance_namara_car(350,0,0,200,1500); osDelay(1000);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, zhua4); osDelay(500);
        move_distance_namara_car(-350,0,0,200,1500);
        WuLiaoData = 0;
        return;
    }

    uint8_t placed_pos = 0xFF, placed_content = 0, placed_shape = 0;
    int grab_timeout = 0;
    uint8_t is_yellow = 0;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, mid);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1250);
    osDelay(500);
    HuaTai_Z(-700); osDelay(300);

    if (Car_Adjust_All(cylinder) != 0) {
        uint8_t re = grab_recheck();
        if (re == 1 || re == 2) {
            placed_content = re; placed_shape = SHAPE_UNKNOWN;
            if (re == 1) {
                if (round_red < EXPECT_RED_PER_ROUND) {
                    placed_pos = find_empty_in(red_order, 2);
                    if (placed_pos == 0xFF) placed_pos = find_empty_global();
                    if (placed_pos != 0xFF) round_red++;
                } else { placed_content = 3; placed_pos = find_empty_in(fail_order, 5); round_unknown++; }
            } else {
                if (round_blue < EXPECT_BLUE_PER_ROUND) {
                    placed_pos = find_blue_cyl_pos();
                    if (placed_pos != 0xFF) round_blue++;
                } else { placed_content = 3; placed_pos = find_empty_in(fail_order, 5); round_unknown++; }
            }
            round_total++;
        } else if (re == 3) { is_yellow = 1; }
        else {
            uint8_t inferred = infer_color();
            if (inferred == 1 && round_red < EXPECT_RED_PER_ROUND) {
                placed_content = 1; placed_shape = SHAPE_UNKNOWN;
                placed_pos = find_empty_in(red_order, 2);
                if (placed_pos != 0xFF) round_red++;
            } else if (inferred == 2 && round_blue < EXPECT_BLUE_PER_ROUND) {
                placed_content = 2; placed_shape = SHAPE_UNKNOWN;
                placed_pos = find_blue_cyl_pos();
                if (placed_pos != 0xFF) round_blue++;
            } else {
                placed_content = 3; placed_shape = SHAPE_UNKNOWN;
                placed_pos = find_empty_in(fail_order, 5);
                round_unknown++;
            }
            round_total++;
        }
        if (placed_pos == 0xFF) placed_pos = find_empty_global();
        if (placed_pos == 0xFF && !is_yellow) return;
    } else {
        osDelay(500);
        while(1) {
            osDelay(50);
            if (++grab_timeout > 200) {
                uint8_t re = grab_recheck();
                if (re == 1 || re == 2) {
                    placed_content = re; placed_shape = SHAPE_UNKNOWN;
                    if (re == 1) {
                        if (round_red < EXPECT_RED_PER_ROUND) {
                            placed_pos = find_empty_in(red_order, 2);
                            if (placed_pos == 0xFF) placed_pos = find_empty_global();
                            if (placed_pos != 0xFF) round_red++;
                        } else { placed_content = 3; placed_pos = find_empty_in(fail_order, 5); round_unknown++; }
                    } else {
                        if (round_blue < EXPECT_BLUE_PER_ROUND) {
                            placed_pos = find_blue_cyl_pos();
                            if (placed_pos != 0xFF) round_blue++;
                        } else { placed_content = 3; placed_pos = find_empty_in(fail_order, 5); round_unknown++; }
                    }
                    round_total++;
                } else if (re == 3) { is_yellow = 1; }
                else {
                    uint8_t inferred = infer_color();
                    if (inferred == 1 && round_red < EXPECT_RED_PER_ROUND) {
                        placed_content = 1; placed_shape = SHAPE_UNKNOWN;
                        placed_pos = find_empty_in(red_order, 2);
                        if (placed_pos != 0xFF) round_red++;
                    } else if (inferred == 2 && round_blue < EXPECT_BLUE_PER_ROUND) {
                        placed_content = 2; placed_shape = SHAPE_UNKNOWN;
                        placed_pos = find_blue_cyl_pos();
                        if (placed_pos != 0xFF) round_blue++;
                    } else {
                        placed_content = 3; placed_shape = SHAPE_UNKNOWN;
                        placed_pos = find_empty_in(fail_order, 5);
                        round_unknown++;
                    }
                    round_total++;
                }
                if (placed_pos == 0xFF) placed_pos = find_empty_global();
                if (placed_pos == 0xFF && !is_yellow) return;
                break;
            }

            if (WuLiaoData == 1 || WuLiaoData == 2) {
                if (round_red >= EXPECT_RED_PER_ROUND) {
                    placed_content = 3; placed_shape = SHAPE_UNKNOWN;
                    placed_pos = find_empty_in(fail_order, 5);
                    if (placed_pos == 0xFF) placed_pos = find_empty_global();
                    round_unknown++;
                } else {
                    placed_content = 1; placed_shape = get_shape_from_wuliao(WuLiaoData);
                    placed_pos = find_empty_in(red_order, 2);
                    if (placed_pos == 0xFF) placed_pos = find_empty_global();
                    if (placed_pos != 0xFF) round_red++;
                }
                round_total++; break;
            } else if (WuLiaoData == 3) {
                if (round_blue >= EXPECT_BLUE_PER_ROUND) {
                    placed_content = 3; placed_shape = SHAPE_UNKNOWN;
                    placed_pos = find_empty_in(fail_order, 5);
                    if (placed_pos == 0xFF) placed_pos = find_empty_global();
                    round_unknown++;
                } else {
                    placed_content = 2; placed_shape = SHAPE_CYLINDER;
                    placed_pos = find_blue_cyl_pos();
                    if (placed_pos != 0xFF) round_blue++;
                }
                round_total++; break;
            } else if (WuLiaoData == 4) {
                if (round_blue >= EXPECT_BLUE_PER_ROUND) {
                    placed_content = 3; placed_shape = SHAPE_UNKNOWN;
                    placed_pos = find_empty_in(fail_order, 5);
                    if (placed_pos == 0xFF) placed_pos = find_empty_global();
                    round_unknown++;
                } else {
                    placed_content = 2; placed_shape = SHAPE_CUBE;
                    placed_pos = find_blue_cube_pos();
                    if (placed_pos != 0xFF) round_blue++;
                }
                round_total++; break;
            } else if (WuLiaoData == 5) { is_yellow = 1; break; }
        }
        if (placed_pos == 0xFF && !is_yellow) return;
    }

    if (is_yellow) {
        change_direction(0); osDelay(500);
        change_direction(0); osDelay(500);
        move_distance_namara_car(350,0,0,200,1200); osDelay(1000);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, zhua4); osDelay(500);
        move_distance_namara_car(-350,0,0,200,1200);
        WuLiaoData = 0;
        return;
    }

    grab_count++;
    if (grab_count > EXPECT_TOTAL_PER_ROUND) { round_check_and_reset(); grab_count = 1; }

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua2);
    HuaTai_Z(14100); osDelay(1800);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua1); osDelay(600);
    HuaTai_Z(-14200); osDelay(1800);
    place_at(placed_pos, placed_content, placed_shape); osDelay(1500);

    if (WuLiaoData == 1 || WuLiaoData == 3) HuaTai_Z(2950);
    else HuaTai_Z(2250);
    osDelay(1200);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua2); osDelay(300);
    HuaTai_Z(-3200); osDelay(500);
}

void put1(int PutmaterialNum) {
    uint8_t target_color = (PutmaterialNum == 1) ? 1 : 2;
    uint8_t found_pos[2] = {0xFF, 0xFF};
    uint8_t found_cnt = 0, i, k;

    // 1. 圆柱优先
    for (i = 0; i < POS_NUM && found_cnt < 2; i++)
        if (pos_used[i] && pos_content[i] == target_color && pos_shape[i] == SHAPE_CYLINDER)
            found_pos[found_cnt++] = i;
    // 2. 方块次之
    for (i = 0; i < POS_NUM && found_cnt < 2; i++) {
        if (!pos_used[i] || pos_content[i] != target_color || pos_shape[i] != SHAPE_CUBE) continue;
        uint8_t dup = 0; for (k = 0; k < found_cnt; k++) if (found_pos[k] == i) { dup = 1; break; }
        if (!dup) found_pos[found_cnt++] = i;
    }
    // 3. 未知最后
    for (i = 0; i < POS_NUM && found_cnt < 2; i++) {
        if (!pos_used[i] || pos_content[i] != target_color || pos_shape[i] != SHAPE_UNKNOWN) continue;
        uint8_t dup = 0; for (k = 0; k < found_cnt; k++) if (found_pos[k] == i) { dup = 1; break; }
        if (!dup) found_pos[found_cnt++] = i;
    }

    // 4. 推理补足
    if (found_cnt < 2) {
        uint8_t other_cnt = 0;
        for (i = 0; i < POS_NUM; i++)
            if (pos_used[i] && pos_content[i] != target_color && pos_content[i] != 3) other_cnt++;
        if (other_cnt >= 2) {
            for (i = 0; i < POS_NUM && found_cnt < 2; i++) {
                if (!pos_used[i] || pos_content[i] != 3 || pos_shape[i] != SHAPE_CYLINDER) continue;
                found_pos[found_cnt++] = i;
            }
            for (i = 0; i < POS_NUM && found_cnt < 2; i++) {
                if (!pos_used[i] || pos_content[i] != 3 || pos_shape[i] == SHAPE_CYLINDER) continue;
                found_pos[found_cnt++] = i;
            }
        }
    }

    // 5. 重识别
    if (found_cnt < 2) {
        uint8_t has_unknown = 0;
        for (i = 0; i < POS_NUM; i++) if (pos_used[i] && pos_content[i] == 3) { has_unknown = 1; break; }
        if (has_unknown) {
            SetMaixCamMode(MAIXCAM_MODE_WULIAO); osDelay(300);
            for (i = 0; i < POS_NUM; i++) {
                if (pos_used[i] && pos_content[i] == 3) {
                    uint8_t real = put_recheck_pos(i);
                    if (real == 1 || real == 2) {
                        pos_content[i] = real;
                        if (real == target_color && found_cnt < 2) found_pos[found_cnt++] = i;
                    }
                }
            }
        }
    }

    // 6. 终极兜底
    if (found_cnt < 2) {
        for (i = 0; i < POS_NUM && found_cnt < 2; i++) {
            if (!pos_used[i]) continue;
            uint8_t dup = 0; for (k = 0; k < found_cnt; k++) if (found_pos[k] == i) { dup = 1; break; }
            if (!dup) found_pos[found_cnt++] = i;
        }
    }

    if (found_cnt == 0) return;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, mid); osDelay(500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua3); osDelay(100);
    Car_Adjust_All(Annulus); osDelay(500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua2); osDelay(500);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pos_servo[found_pos[0]]);
    pos_used[found_pos[0]] = pos_content[found_pos[0]] = pos_shape[found_pos[0]] = 0;
    osDelay(1000); HuaTai_Z(3650); osDelay(1500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua1); osDelay(500);
    HuaTai_Z(-3850); osDelay(1500);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, mid); osDelay(2000);
    HuaTai_Z(13450); osDelay(2000);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua2); osDelay(600);
    HuaTai_Z(-13600); osDelay(1500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua3); osDelay(100);

    if (found_cnt >= 2) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua2);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pos_servo[found_pos[1]]);
        pos_used[found_pos[1]] = pos_content[found_pos[1]] = pos_shape[found_pos[1]] = 0;
        osDelay(1000); HuaTai_Z(3650); osDelay(1500);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua1); osDelay(500);
        HuaTai_Z(-3850); osDelay(1500);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, mid); osDelay(2000);
        HuaTai_Z(10800); osDelay(2000);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua2); osDelay(600);
        HuaTai_Z(-11000); osDelay(1500);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua3); osDelay(100);
    }
}

/* ==================== 奖杯主逻辑 ==================== */
static void newgrab1_task_reset(void) {
    memset(g_vote6, 0, sizeof(g_vote6)); memset(g_vote7, 0, sizeof(g_vote7));
    memset(g_vote8, 0, sizeof(g_vote8)); memset(g_total, 0, sizeof(g_total));
    memset(g_final_val, 0, sizeof(g_final_val)); memset(g_target_rank, 0xFF, sizeof(g_target_rank));
    g_phys_pos[0] = POS_6; g_phys_pos[1] = POS_7; g_phys_pos[2] = POS_8;
    g_result_valid = 0;
}

static void record_votes(int idx, int v6, int v7, int v8) {
    g_vote6[idx] = v6; g_vote7[idx] = v7; g_vote8[idx] = v8;
    g_total[idx] = v6 + v7 + v8;
}

static float get_conf(int cup, int val) {
    if (g_total[cup] == 0) return 0;
    if (val == 6) return (float)g_vote6[cup] / g_total[cup];
    if (val == 7) return (float)g_vote7[cup] / g_total[cup];
    if (val == 8) return (float)g_vote8[cup] / g_total[cup];
    return 0;
}

static uint8_t get_confident_val(int cup) {
    if (g_total[cup] == 0) return 0;
    int maxCnt, secondCnt, maxVal;
    if (g_vote6[cup] >= g_vote7[cup]) { maxCnt = g_vote6[cup]; secondCnt = g_vote7[cup]; maxVal = 6; }
    else { maxCnt = g_vote7[cup]; secondCnt = g_vote6[cup]; maxVal = 7; }
    if (g_vote8[cup] > maxCnt) { secondCnt = maxCnt; maxCnt = g_vote8[cup]; maxVal = 8; }
    else if (g_vote8[cup] > secondCnt) { secondCnt = g_vote8[cup]; }

    float highRatio = (float)maxCnt / g_total[cup];
    float diffRatio = (float)(maxCnt - secondCnt) / g_total[cup];
    if (highRatio >= SINGLE_RATIO_HIGH && diffRatio >= SINGLE_RATIO_DIFF) return (uint8_t)maxVal;
    return 0;
}

static int check_rank_permutation(void) {
    int seen[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++) {
        uint8_t r = g_target_rank[i];
        if (r > 2 || seen[r]) return 0;
        seen[r] = 1;
    }
    return 1;
}

static void global_assign(void) {
    const int perms[6][3] = {{6,7,8}, {6,8,7}, {7,6,8}, {7,8,6}, {8,6,7}, {8,7,6}};
    float best_score = -1; int best_idx = 0;
    for (int i = 0; i < 6; i++) {
        float score = get_conf(0, perms[i][0]) + get_conf(1, perms[i][1]) + get_conf(2, perms[i][2]);
        if (score > best_score) { best_score = score; best_idx = i; }
    }
    for (int i = 0; i < 3; i++) g_final_val[i] = perms[best_idx][i];

    g_result_valid = 1;
    for (int i = 0; i < 3; i++) if (g_total[i] == 0) g_result_valid = 0;
    if (best_score < MIN_TOTAL_SCORE) g_result_valid = 0;
    for (int i = 0; i < 3; i++) {
        float c = get_conf(i, g_final_val[i]);
        if (c < MIN_MAX_CONF) g_result_valid = 0;
    }
}

static void fallback_assign(void) {
    uint8_t confident[3] = {0, 0, 0}; int used_val[3] = {0, 0, 0}; int confident_cnt = 0;
    for (int i = 0; i < 3; i++) {
        uint8_t v = get_confident_val(i);
        if (v != 0 && !used_val[v - 6]) { confident[i] = v; used_val[v - 6] = 1; confident_cnt++; }
    }
    if (confident_cnt == 3) {
        for (int i = 0; i < 3; i++) g_target_rank[i] = confident[i] - 6;
    } else if (confident_cnt == 2) {
        int remain_val = 0; for (int v = 0; v < 3; v++) if (!used_val[v]) { remain_val = v + 6; break; }
        for (int i = 0; i < 3; i++) g_target_rank[i] = (confident[i] != 0) ? (confident[i] - 6) : (remain_val - 6);
    } else if (confident_cnt == 1) {
        int remain_val[2] = {0, 0}; int idx = 0;
        for (int v = 0; v < 3; v++) if (!used_val[v]) remain_val[idx++] = v + 6;
        int ri = 0;
        for (int i = 0; i < 3; i++) g_target_rank[i] = (confident[i] != 0) ? (confident[i] - 6) : (remain_val[ri++] - 6);
    } else {
        for (int i = 0; i < 3; i++) g_target_rank[i] = g_phys_pos[i];
    }
}

void newgrab1(int materialNum1) {
    const int SAMPLE_PER_ROUND = 25; const int ROUND_MAX = 5;
    int total6 = 0, total7 = 0, total8 = 0, dat;

    if (materialNum1 == 0) newgrab1_task_reset();

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, mid);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua3); osDelay(300);
    Car_Adjust_All(cylinder); osDelay(200);

    for (int round = 0; round < ROUND_MAX; round++) {
        for (int i = 0; i < SAMPLE_PER_ROUND; i++) {
            osDelay(20); dat = WuLiaoData;
            if (dat == 6) total6++; else if (dat == 7) total7++; else if (dat == 8) total8++;
        }
        int t = total6 + total7 + total8, mx = total6;
        if (total7 > mx) mx = total7; if (total8 > mx) mx = total8;
        if (t > 0 && (float)mx / t >= 0.60f) break;
        osDelay(50);
    }

    record_votes(materialNum1, total6, total7, total8);
    uint8_t target_phys = POS_6 + materialNum1;
    g_phys_pos[materialNum1] = target_phys;

    osDelay(300); __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua2); osDelay(200);
    HuaTai_Z(13800); osDelay(1500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua1); osDelay(500);
    HuaTai_Z(-14000); osDelay(1500); HuaTai_X(-4000); osDelay(200);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, trophy_servo[target_phys]); osDelay(1500);
    HuaTai_X(4000); osDelay(500); HuaTai_Z(2850); osDelay(1500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua2); osDelay(800);
    HuaTai_Z(-3000); osDelay(800);

    if (materialNum1 == 2) {
        global_assign();
        if (g_result_valid) {
            for (int i = 0; i < 3; i++) g_target_rank[i] = g_final_val[i] - 6;
        } else {
            fallback_assign();
            if (!check_rank_permutation()) for (int i = 0; i < 3; i++) g_target_rank[i] = g_phys_pos[i];
        }
    }
}

void newput1(int rank) {
    if (rank < 0 || rank > 2) return;
    int cup_idx = -1;
    for (int i = 0; i < 3; i++) if (g_target_rank[i] == rank) { cup_idx = i; break; }
    if (cup_idx == -1) return;

    uint8_t src = g_phys_pos[cup_idx];
    int place_height, retract_height, need_final_x = 0;

    if (rank == 0) { place_height = 9700; retract_height = 9900; }
    else if (rank == 1) { place_height = 11400; retract_height = 11600; }
    else if (rank == 2) { place_height = 12800; retract_height = 13000; need_final_x = 1; }

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua3); osDelay(200);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, mid); osDelay(500);
    if (rank == 1) { HuaTai_X(-7000); HAL_Delay(1500); }

    // 校准根据暂存位
    if (src == POS_6) Car_Adjust_All(Annulus2);
    else if (src == POS_7) Car_Adjust_All(Annulus3);
    else if (src == POS_8) Car_Adjust_All(Annulus6); // 假设使用 Annulus6

    osDelay(500); __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua2); osDelay(100);
    HuaTai_X(7000); osDelay(800);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, trophy_servo[src]); osDelay(1000);

    HuaTai_Z(3450); osDelay(1500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua1); osDelay(1000);
    HuaTai_Z(-3600); osDelay(500); HuaTai_X(-7000); osDelay(1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, mid); osDelay(1500);

    HuaTai_Z(place_height); osDelay(2000);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua2); osDelay(500);
    HuaTai_Z(-retract_height); osDelay(2500);

    if (need_final_x) { HuaTai_X(7000); osDelay(1300); }
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, zhua3);
}