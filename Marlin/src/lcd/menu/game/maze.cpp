/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(MARLIN_MAZE)

#include "game.h"

int8_t move_dir, last_move_dir, // NESW0//NESW0
       prizex, prizey, prize_cnt, old_encoder;
fixed_t playerx, playery;

// Up to 50 lines, then you win!//最多50行，你就赢了！
typedef struct { int8_t x, y; } pos_t;
uint8_t head_ind;
pos_t maze_walls[50] = {
  { 0, 0 }
};

// Turn the player cw or ccw//将播放机顺时针或逆时针转动
inline void turn_player(const bool cw) {
  if (move_dir == 4) move_dir = last_move_dir;
  move_dir += cw ? 1 : -1;
  move_dir &= 0x03;
  last_move_dir = move_dir;
}

// Reset the player for a new game//为新游戏重置玩家
void player_reset() {
  // Init position//初始位置
  playerx = BTOF(1);
  playery = BTOF(GAME_H / 2);

  // Init motion with a ccw turn//逆时针转弯初始运动
  move_dir = 0;
  turn_player(false);

  // Clear prize flag//清除奖旗
  prize_cnt = 255;

  // Clear the controls//清除控件
  ui.encoderPosition = 0;
  old_encoder = 0;
}

void MazeGame::game_screen() {
  // Run the sprite logic//运行sprite逻辑
  if (game_frame()) do {     // Run logic twice for finer resolution//运行逻辑两次以获得更高的分辨率

    // Move the man one unit in the current direction//沿当前方向将该人员移动一个单位
    // Direction index 4 is for the stopped man//方向索引4用于被拦住的人
    const int8_t oldx = FTOB(playerx), oldy = FTOB(playery);
    pos_t dir_add[] = { { 0, -1 }, { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, 0 } };
    playerx += dir_add[move_dir].x;
    playery += dir_add[move_dir].y;
    const int8_t x = FTOB(playerx), y = FTOB(playery);

  } while(0);

  u8g.setColorIndex(1);

  // Draw Score//平局
  if (PAGE_UNDER(HEADER_H)) lcd_put_int(0, HEADER_H - 1, score);

  // Draw the maze//画迷宫
  // LOOP_L_N(n, head_ind) {//回路（N，头部）{
  //   const pos_t &p = maze_walls[n], &q = maze_walls[n + 1];//const pos_t&p=迷宫墙[n]，&q=迷宫墙[n+1]；
  //   if (p.x == q.x) {//如果（p.x==q.x）{
  //     const int8_t y1 = GAMEY(_MIN(p.y, q.y)), y2 = GAMEY(_MAX(p.y, q.y));//常数int8=GAMEY（_MIN（p.y，q.y）），y2=GAMEY（_MAX（p.y，q.y））；
  //     if (PAGE_CONTAINS(y1, y2))//如果（第_页包含（y1，y2））
  //       u8g.drawVLine(GAMEX(p.x), y1, y2 - y1 + 1);//u8g.绘制线（GAMEX（p.x），y1，y2-y1+1）；
  //   }//   }
  //   else if (PAGE_CONTAINS(GAMEY(p.y), GAMEY(p.y))) {//else if（第_页包含（GAMEY（p.y）、GAMEY（p.y）））{
  //     const int8_t x1 = GAMEX(_MIN(p.x, q.x)), x2 = GAMEX(_MAX(p.x, q.x));//常数int8 x1=GAMEX（_MIN（p.x，q.x）），x2=GAMEX（_MAX（p.x，q.x））；
  //     u8g.drawHLine(x1, GAMEY(p.y), x2 - x1 + 1);//u8g.drawHLine（x1，GAMEY（p.y），x2-x1+1）；
  //   }//   }
  // }// }

  // Draw Man//绘图员
  // const int8_t fy = GAMEY(foody);//const int8_t fy=GAMEY（foody）；
  // if (PAGE_CONTAINS(fy, fy + FOOD_WH - 1)) {//如果（第页包含（fy，fy+食品WH-1））{
  //   const int8_t fx = GAMEX(foodx);//const int8_t fx=GAMEX（foodx）；
  //   u8g.drawFrame(fx, fy, FOOD_WH, FOOD_WH);//u8g.牵引架（fx、fy、FOOD_WH、FOOD_WH）；
  //   if (FOOD_WH == 5) u8g.drawPixel(fx + 2, fy + 2);//如果（食物WH==5）u8g.drawPixel（fx+2，fy+2）；
  // }// }

  // Draw Ghosts//画鬼魂
  // const int8_t fy = GAMEY(foody);//const int8_t fy=GAMEY（foody）；
  // if (PAGE_CONTAINS(fy, fy + FOOD_WH - 1)) {//如果（第页包含（fy，fy+食品WH-1））{
  //   const int8_t fx = GAMEX(foodx);//const int8_t fx=GAMEX（foodx）；
  //   u8g.drawFrame(fx, fy, FOOD_WH, FOOD_WH);//u8g.牵引架（fx、fy、FOOD_WH、FOOD_WH）；
  //   if (FOOD_WH == 5) u8g.drawPixel(fx + 2, fy + 2);//如果（食物WH==5）u8g.drawPixel（fx+2，fy+2）；
  // }// }

  // Draw Prize//抽奖
  // if (PAGE_CONTAINS(prizey, prizey + PRIZE_WH - 1)) {//如果（页面包含（prizey，prizey+PRIZE_WH-1））{
  //   u8g.drawFrame(prizex, prizey, PRIZE_WH, PRIZE_WH);//u8g.牵引架（prizex、prizey、PRIZE\u-WH、PRIZE\u-WH）；
  //   if (PRIZE_WH == 5) u8g.drawPixel(prizex + 2, prizey + 2);//如果（PRIZE_WH==5）u8g.drawPixel（prizex+2，prizey+2）；
  // }// }

  // Draw GAME OVER//平局
  if (!game_state) draw_game_over();

  // A click always exits this game//单击始终退出此游戏
  if (ui.use_click()) exit_game();
}

void MazeGame::enter_game() {
  init_game(1, game_screen); // Game running//比赛跑
  reset_player();
  reset_enemies();
}

#endif // MARLIN_MAZE//马林湖迷宫
