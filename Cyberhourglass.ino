#include <Wire.h>  
#include <MPU6050_tockn.h>
#include "Arduino.h"
#include "LedControl.h"
#include "Delay.h"

MPU6050 mpu6050(Wire);
#define MATRIX_A 1
#define MATRIX_B 0
// 点阵屏驱动引脚
#define PIN_DATAIN 3  // DIN 引脚
#define PIN_CLK 2     // CLK 引脚
#define PIN_LOAD 1    // CS 引脚
// 点阵屏安装方向
#define ROTATION_OFFSET 90
#define Gravity 6
//定义粒子实体数
#define PARTICLE_COUNT 64

//定义粒子结构
struct Particle{
  int x;  // 粒子的x坐标，float
  int y;  // 粒子的y坐标，float
  float velX;  // 粒子的x方向速度，float
  float velY;  // 粒子的y方向速度，float
  bool active;  // 粒子是否激活，bool
};

// 定义粒子数组，包含其位置和速度信息
Particle particles[PARTICLE_COUNT];  
//定义点阵屏对象，包含两个实体设备
LedControl lc = LedControl(PIN_DATAIN, PIN_CLK, PIN_LOAD, 2);
// 定义一个16*16的二维数组，用于记录点阵屏上的粒子位置，实际有效区域为x:0-7, y:8-15以及x:8-15, y:0-7
bool grid[16][16] = {0};  

void ParticleInit();
void ParticleUpdate();
void ParticleCollision(Particle *p);
void ParticleMove(Particle *p);
void ParticleRender();


// 获取当前的重力方向
int getGravity() {
  mpu6050.update();
  float x = mpu6050.getAccX();  // 获取x方向的加速度
  float y = mpu6050.getAccY();  // 获取y方向的加速度
  if (y > 0.8) {
    return 180;  // 重力方向朝下
  } else if (x > 0.8) {
    return 90;   // 重力方向朝左
  } else if (y < -0.8) {
    return 0;    // 
  } else if (x < -0.8) {
    return 270;  // 重力方向朝右
  } else {
    return 0; // 其他情况保持上一次的方向
  }
}

void setup() {
  Serial.begin(9600);
  Wire.setSDA(12);
  Wire.setSCL(13);
  Wire.begin();
  mpu6050.begin();
  randomSeed(analogRead(14));
  for (byte i = 0; i < 2; i++) {
    lc.shutdown(i, false);
    lc.setIntensity(i, 2);  // 控制粒子的亮度强度
  }
  ParticleInit();
  ParticleRender();
}

void loop() {
  delay(50);
  mpu6050.update();
  float y = mpu6050.getAccY();
  Serial.println("y方向加速度：");
  Serial.println(y);
  ParticleUpdate();
  ParticleRender();
}

/**
* @brief 初始化粒子,使用点阵屏参考坐标系，点亮上屏所有点
* 
*/
void ParticleInit(){
  int i = 0;
  for(int x = 0;x<8;x++){
    for(int y = 8;y<16;y++){
      particles[i].x = x;
      particles[i].y = y;
      particles[i].velX = 0;
      particles[i].velY = 0;
      particles[i].active = true;
      grid[x][y] = 1;
      i++;
    }
  }
}
/**
 * @brief 更新粒子位置，根据加速度计算粒子速度
 * 
 */
void ParticleUpdate(){
  mpu6050.update();
  //获取全局坐标系下的加速度
  float g_gx = -mpu6050.getAccX()*Gravity;  
  float g_gy = mpu6050.getAccY()*Gravity;  

  //获取局部(LED)坐标系下的加速度
  float l_gx = -g_gx * 0.707 + g_gy * 0.707;
  float l_gy = -g_gx * 0.707 - g_gy * 0.707;
  //遍历方式更新粒子位置
  for(int i = 0;i<PARTICLE_COUNT;i++){
    particles[i].velX = l_gx;
    particles[i].velY = l_gy;
    //ParticleCollision(&particles[i]);
    Serial.println("初始化速度");
    Serial.println(g_gx);
    Serial.println(g_gy);
    ParticleMove(&particles[i]);
  }
}
/**
 * @brief 检测粒子碰撞墙壁或粒子，撞到则速度为0,对于两点阵屏连接处不处理
 * 
 * @param p 粒子实体指针
 */
void ParticleCollision(Particle *p){
  if(((p->x == 7) && (p->y == 0)) || ((p->x == 8) && (p->y == -1))){
    return;
  }
  if(p->velX < 0){
    if((particles->x == 0 && (8<=particles->y<=15)) || (particles->x == 8 && (0<=particles->y<=7))){
      p->velX = 0;
    }
  }
  if(p->velX > 0){
    if((particles->x == 7 && (8<=particles->y<=15)) || (particles->x == 15 && (0<=particles->y<=7))){
      p->velX = 0;
    }
  }
  if(p->velY < 0){
    if((particles->y == 8 && (0<=particles->x<=7)) || (particles->y == 0 && (8<=particles->x<=15))){
      p->velY = 0;
    }
  }
  if(p->velY > 0){
    if((particles->y == 15 && (0<=particles->x<=7)) || (particles->y == 7 && (8<=particles->x<=15))){
      p->velY = 0;
    }
  }
}
/**
 * @brief 粒子移动后位置计算,内部更新粒子位置以及网表
 * 
 * @param p 粒子实体指针
 */
void ParticleMove(Particle *p){
//存储粒子速度绝对值以及符号
  int dx = ceil(abs(p->velX));
  int dy = ceil(abs(p->velY));
  int signx = (p->velX>0)?1:-1;
  int signy = (p->velY>0)?1:-1;
//存储最大速度方向，1为x方向，0为y方向
  bool maxer = (max(dx,dy) == dx)?true:false;
//存储最大与最小之比(就近取整)
  float scale = (maxer)?(dx/dy):(dy/dx);
  float count = 0;
  Serial.println("取整速度：");
  Serial.print("dx:");
  Serial.println(dx);
  Serial.print("dy:");
  Serial.println(dy);
//限制增量大小，防止粒子过墙,对于连接处如果移动方向合适直接移动至另一屏并清空速度
  if((p->x == 7) && (p->y == 8)){
    if ((signx > 0) && (signy <0) && (grid[8][7] == 0)){
      grid[8][7] =1;
      grid[7][8] = 0;
      p->x = 8;
      p->y = 7;
      p->velX = 0;
      p->velY = 0;
      return;
    }
  }
  else if((p->x == 8) && (p->y == 7)){
    if((signx < 0) && (signy > 0) && (grid[7][8] == 0)){
      grid[7][8] = 1;
      grid[8][7] = 0;
      p->x = 7;
      p->y = 8;
      p->velX = 0;
      p->velY = 0;
      return;
    }
  }
  else if(p->x < 8){
    dx = ((signx*dx+p->x)>7)?(7-p->x):(dx);
    dx = ((signx*dx+p->x)<0)?(p->x):(dx);
    dy = ((signy*dy+p->y)>15)?(15-p->y):(dy);
    dy = ((signy*dy+p->y)<8)?(p->y-8):(dy);
  }
  else{
    dx = ((signx*dx+p->x)>15)?(15-p->x):(dx);
    dx = ((signx*dx+p->x)<8)?(p->x-8):(dx);
    dy = ((signy*dy+p->y)>7)?(7-p->y):(dy);
    dy = ((signy*dy+p->y)<0)?(p->y):(dy);
  }
  //打印限制后的粒子速度
  Serial.println("限制速度：");
  Serial.print("dx:");
  Serial.println(dx);
  Serial.print("dy:");
  Serial.println(dy);
//移动粒子
  while((dx>=0) && (dy>=0)){
    if(grid[p->x+signx*dx][p->y+signy*dy] == 0){
      grid[p->x+signx*dx][p->y+signy*dy] = 1;
      grid[p->x][p->y] = 0;
      p->x += signx*dx;
      p->y += signy*dy;
      break;
    }
    if(maxer){
      dx--;
      count++;
      if(count >= scale){
        dy--;
        count -= scale ;
      }
    }
    else{
      dy--;
      count++;
      if(count >= scale){
        dx--;
        count -= scale;
      }
    }
  }
}
void ParticleRender(){
  //上屏区渲染
  for(int i=7;i>=0;i--){
    for(int j=8;j<16;j++){
      if(grid[i][j] == 1){
        lc.setLed(0,(15-j),(i),true);
      }
      else{
        lc.setLed(0,(15-j),(i),false);
      }
    }
  }
  //下屏区渲染
  for(int i=15;i>=8;i--){
    for(int j=0;j<8;j++){
      if(grid[i][j] == 1){
        lc.setLed(1,(7-j),(i-8),true);
      }
      else{
        lc.setLed(1,(7-j),(i-8),false);
      }
    }
  }
}