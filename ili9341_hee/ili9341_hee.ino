//***************************************************************************************************************************************
/* Librería para el uso de la pantalla ILI9341 en modo 8 bits
   Basado en el código de martinayotte - https://www.stm32duino.com/viewtopic.php?t=637
   Adaptación, migración y creación de nuevas funciones: Pablo Mazariegos y José Morales
   Con ayuda de: José Guerra
   IE3027: Electrónica Digital 2 - 2019
*/
//***************************************************************************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <TM4C123GH6PM.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include "bitmaps.h"
#include "font.h"
#include "lcd_registers.h"

#define LCD_RST PD_0
#define LCD_CS PD_1
#define LCD_RS PD_2
#define LCD_WR PD_3
#define LCD_RD PE_1


#include <SPI.h>
#include <SD.h>

File myFile;

#define player2 PUSH1
#define player1 PUSH2
#define down 0
#define up 1
int DPINS[] = {PB_0, PB_1, PB_2, PB_3, PB_4, PB_5, PB_6, PB_7};
uint16_t t = 0;
uint16_t t2 = 0;
uint16_t kirby[] = {0, 0}; //posicion actual de kirby
uint16_t last[] = {0, 0}; //posicion inicial
uint8_t kirby_d[] = {8, 8}; //tamaño de kirby
uint8_t kirby_anim = 1; //animacion actual de kirby
bool kirby_jump = true; //permite saltar a kirby
bool move_kirby = up; //indica el movimiento
uint16_t kirby2[] = {0, 0}; //posicion actual de kirby2
uint8_t kirby_d2[] = {8, 8}; //tamaño de kirby2
uint8_t kirby_anim2 = 1; //animacion actual de kirby2
bool kirby_jump2 = true; //permite saltar a kirby2
bool move_kirby2 = up; //indica el movimiento 2
uint8_t ground = 207; //nivel del piso
String jugador1 = "Jugador 1";
String jugador2 = "Jugador 2";
//***************************************************************************************************************************************
// coronavirus
//***************************************************************************************************************************************
uint16_t objeto1[] = {0, ground - 26}; //objeto 1
uint8_t do1[] = {12, 13};
uint16_t objeto2[] = {420 - t, 179}; //objeto 2
uint8_t do2[] = {12, 13};
uint16_t objeto3[] = {0, ground - 26}; //objeto 3
uint16_t objeto4[] = {420 - t, 179}; //objeto 4
//***************************************************************************************************************************************
// Functions Prototypes
//***************************************************************************************************************************************
void LCD_Init(void);
void LCD_CMD(uint8_t cmd);
void LCD_DATA(uint8_t data);
void SetWindows(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);
void LCD_Clear(unsigned int c);
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void Rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
void LCD_Print(String text, int x, int y, int fontSize, int color, int background);

void LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[]);
void LCD_Sprite(int x, int y, int width, int height, unsigned char bitmap[], int columns, int index, char flip, char offset);

uint8_t collider(uint16_t objeto1[], uint16_t objeto2[], uint8_t d1[] , uint8_t d2[]);
void kirby_spriter(uint8_t animacion, int anim, bool lado);
void kirby_spriter2(uint8_t animacion, int anim, bool lado);
void jumping(uint16_t personaje[], uint8_t numero, uint8_t *animacion, bool *jump_available, bool *move_y, uint16_t piso);
uint8_t kirby_collision(uint8_t caso, uint8_t anim);
uint8_t kirby2_collision(uint8_t caso, uint8_t anim);

unsigned char Char_uChar(char value);
void SD_Bitmap(int x, int y, int width, int height, char *filename);
//***************************************************************************************************************************************
// Inicialización
//***************************************************************************************************************************************
void setup() {
  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  Serial.begin(9600);
  GPIOPadConfigSet(GPIO_PORTB_BASE, 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
  Serial.println("Inicio");
  LCD_Init();
  LCD_Clear(0x00);

  pinMode(player1, INPUT_PULLUP);
  pinMode(player2, INPUT_PULLUP);

  SPI.setModule(0);
  Serial.print("Initializing SD card...");
  pinMode(10, OUTPUT);
  if (!SD.begin(32)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  myFile = SD.open("/");
}
//***************************************************************************************************************************************
// Loop Infinito
//***************************************************************************************************************************************
void loop() {
  LCD_Clear (0xffff);
  String text1 = "Corona Time!";
  LCD_Print(text1, 69, 83, 2, 0xffff, 0x421b);
  String text2 = "Presiona para comenzar";
  LCD_Print(text2, 80, 100, 1, 0x421b, 0xffff);
  //LCD_Sprite(int x, int y, int width, int height, unsigned char bitmap[],int columns, int index, char flip, char offset);

  //LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[]);
  /*LCD_Bitmap(50, 50, 16, 16, kirby_standing);
    LCD_Bitmap(50, 100, 64, 16, kirby_running);
    LCD_Bitmap(50, 150, 16, 16, kirby_jumping);
    LCD_Bitmap(50, 150, 160, 16, kirby_vanishing)*/
  for (int x = 0; x < 319; x++) {
    LCD_Bitmap(x, 0, 16, 32, tile);
    LCD_Bitmap(x, 33, 16, 32, tile);

    LCD_Bitmap(x, ground, 16, 16, tile2);
    LCD_Bitmap(x, ground + 16, 16, 16, tile2);
    x += 15;
  }
  kirby[1] = ground - 16;
  uint16_t t = 0;
  while ((digitalRead(player1) == 1)&&(digitalRead(player2)==1)) {
    t++;
    t = t % 346;
    delay(5);
    int anim3 = (t / 16) % 4;
    kirby[0] = 150;
    kirby_spriter(kirby_anim, anim3, 0);
    objeto1[0] = 320 - t;
    LCD_Sprite(objeto1[0], objeto1[1], 26, 26, coronavirus, 1, 0, 0, 0);
    V_line(320 + 25 - t, ground - 27, 26, 0xffff);

    if ((t == 111) && kirby_jump) {
      kirby_anim = 2;
      last[0] = kirby[1];
      kirby_jump = false;
    }
    jumping(kirby, 0, &kirby_anim, &kirby_jump, &move_kirby, ground);
    kirby_anim = kirby_collision(collider(kirby, objeto1, kirby_d, do1), kirby_anim);
  }
  t2=0;
  t=0;
  kirby_jump = true; //permite saltar a kirby2
  move_kirby = up; //indica el movimiento 2
  kirby_jump2 = true; //permite saltar a kirby2
  move_kirby2 = up; //indica el movimiento 2
  ground = 224;
  kirby[0] = 150;
  kirby2[0] = 150;
  kirby[1] = ground - 16;
  objeto1[1] = ground - 26; //objeto 1
  objeto2[1] = ground - 146; //objeto 2
  kirby2[1] = ground - 136;
  kirby_anim = 1;
  kirby_anim2 = 1;
  LCD_Clear (0xffff);
  for (int x = 0; x < 319; x++) {
    LCD_Bitmap(x, 120, 16, 32, tile);
    LCD_Bitmap(x, 0, 16, 32, tile);

    LCD_Bitmap(x, ground, 16, 16, tile2);
    LCD_Bitmap(x, ground - 120, 16, 16, tile2);
    x += 15;
  }
  LCD_Print(jugador2, 5, 33, 1, 0xffff, 0x421b);
  LCD_Print(jugador1, 5, 153, 1, 0xffff, 0x421b);
    for(uint8_t x=0;x<3;x++){
      String muestra="";
      switch(x){
        case 0: muestra="3";
        break;
        case 1: muestra="2";
        break;
        case 2: muestra="1";
        break;
      }
    LCD_Print(muestra, 160, 120, 2, 0xffff, 0x421b);
    delay(1000);
  }
    for (int x = 0; x < 319; x++) {
    LCD_Bitmap(x, 120, 16, 32, tile);
    LCD_Bitmap(x, 0, 16, 32, tile);

    LCD_Bitmap(x, ground, 16, 16, tile2);
    LCD_Bitmap(x, ground - 120, 16, 16, tile2);
    x += 15;
  }
  uint16_t tiempo=5000;
  uint8_t contador2 = 0;
  uint8_t contador = 0;
  uint8_t animar=0;
  while ((kirby_anim!=4)&&(kirby_anim2!=4)) {
    animar++;
    if(contador!=0){
      contador--;
    }
    else if(contador == 0){
      if(t==345){
        contador=random(1,200);
        t=0;
      }else{
        t++;
      }
    }
    if(contador2!=0){
      contador2--;
    }
    else if(contador2 == 0){
      if(t2==345){
        contador2=random(1,200);
        t2=0;
      }else{
        t2++;
      }
    }
    if(tiempo!=0){
      tiempo--;
    }
    delayMicroseconds(tiempo);
    int anim3 = (animar / 16) % 4;
    
    kirby_spriter2(kirby_anim2, anim3, 0);
    objeto2[0] = 320 - t2;
    LCD_Sprite(objeto2[0], objeto2[1], 26, 26, coronavirus, 1, 0, 0, 0);
    V_line(320 + 25 - t2, ground - 147, 26, 0xffff);
    
    kirby_spriter(kirby_anim, anim3, 0);
    objeto1[0] = 320 - t;
    LCD_Sprite(objeto1[0], objeto1[1], 26, 26, coronavirus, 1, 0, 0, 0);
    V_line(320 + 25 - t, ground - 27, 26, 0xffff);

    if (!(digitalRead(player1)) && kirby_jump) {
      kirby_anim = 2;
      last[0] = kirby[1];
      kirby_jump = false;
    }
    jumping(kirby, 0, &kirby_anim, &kirby_jump, &move_kirby, ground);
    kirby_anim = kirby_collision(collider(kirby, objeto1, kirby_d, do1), kirby_anim);

    if (!(digitalRead(player2)) && kirby_jump2) {
      kirby_anim2 = 2;
      last[1] = kirby2[1];
      kirby_jump2 = false;
    }
    jumping(kirby2, 1, &kirby_anim2, &kirby_jump2, &move_kirby2, ground-120);
    kirby_anim2 = kirby_collision(collider(kirby2, objeto2, kirby_d2, do2), kirby_anim2);
    
  }
  while((t!=345)||(t2!=345)){
    animar++;
    int anim3 = (animar / 16) % 4;
    delay(5);
    if(t!=345){
      t++;
    }
    if(t2!=345){
      t2++;
    }
    kirby_spriter2(kirby_anim2, anim3, 0);
    objeto2[0] = 320 - t2;
    LCD_Sprite(objeto2[0], objeto2[1], 26, 26, coronavirus, 1, 0, 0, 0);
    V_line(320 + 25 - t2, ground - 147, 26, 0xffff);
    jumping(kirby, 0, &kirby_anim, &kirby_jump, &move_kirby, ground);
    
    kirby_spriter(kirby_anim, anim3, 0);
    objeto1[0] = 320 - t;
    LCD_Sprite(objeto1[0], objeto1[1], 26, 26, coronavirus, 1, 0, 0, 0);
    V_line(320 + 25 - t, ground - 27, 26, 0xffff);
    jumping(kirby2, 1, &kirby_anim2, &kirby_jump2, &move_kirby2, ground-120);
  }
  String ganador = "Ganador!!";
  uint16_t posicion = 0;
  uint16_t pos_ganador = 0;
  if(kirby_anim!=4){
      kirby_anim=0;
      posicion = ground - 150;
      pos_ganador = 153; 
    }else{
      kirby_anim2=0;
      posicion = ground - 30;
      pos_ganador = 33;
    }
  LCD_Print(ganador, 120, pos_ganador, 2, 0xffff, 0x421b);
  for (int x = 0; x <= 365; x++) {
    delay(10);
    int anim3 = (x / 16) % 4;
    if(kirby_anim==0){
      kirby_spriter(kirby_anim, anim3, 0);
    }else{
      kirby_spriter2(kirby_anim2, anim3, 0);
    }
    LCD_Sprite(x, posicion, 45, 30, ataud, 4, anim3, 0, 0);
    V_line(x - 1, posicion, 30, 0xffff);
  }
  
  SD_Bitmap(0, 0, 320, 240, "graficos.txt");
  delay(5000);
  kirby_anim=1;
  kirby_anim2=1;
  kirby_jump = true; //permite saltar a kirby2
  move_kirby = up; //indica el movimiento 2
  kirby_jump2 = true; //permite saltar a kirby2
  move_kirby2 = up; //indica el movimiento 2
}
//***************************************************************************************************************************************
// Función para inicializar LCD
//***************************************************************************************************************************************
void LCD_Init(void) {
  pinMode(LCD_RST, OUTPUT);
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_WR, OUTPUT);
  pinMode(LCD_RD, OUTPUT);
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(DPINS[i], OUTPUT);
  }
  //****************************************
  // Secuencia de Inicialización
  //****************************************
  digitalWrite(LCD_CS, HIGH);
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_WR, HIGH);
  digitalWrite(LCD_RD, HIGH);
  digitalWrite(LCD_RST, HIGH);
  delay(5);
  digitalWrite(LCD_RST, LOW);
  delay(20);
  digitalWrite(LCD_RST, HIGH);
  delay(150);
  digitalWrite(LCD_CS, LOW);
  //****************************************
  LCD_CMD(0xE9);  // SETPANELRELATED
  LCD_DATA(0x20);
  //****************************************
  LCD_CMD(0x11); // Exit Sleep SLEEP OUT (SLPOUT)
  delay(100);
  //****************************************
  LCD_CMD(0xD1);    // (SETVCOM)
  LCD_DATA(0x00);
  LCD_DATA(0x71);
  LCD_DATA(0x19);
  //****************************************
  LCD_CMD(0xD0);   // (SETPOWER)
  LCD_DATA(0x07);
  LCD_DATA(0x01);
  LCD_DATA(0x08);
  //****************************************
  LCD_CMD(0x36);  // (MEMORYACCESS)
  LCD_DATA(0x40 | 0x80 | 0x20 | 0x08); // LCD_DATA(0x19);
  //****************************************
  LCD_CMD(0x3A); // Set_pixel_format (PIXELFORMAT)
  LCD_DATA(0x05); // color setings, 05h - 16bit pixel, 11h - 3bit pixel
  //****************************************
  LCD_CMD(0xC1);    // (POWERCONTROL2)
  LCD_DATA(0x10);
  LCD_DATA(0x10);
  LCD_DATA(0x02);
  LCD_DATA(0x02);
  //****************************************
  LCD_CMD(0xC0); // Set Default Gamma (POWERCONTROL1)
  LCD_DATA(0x00);
  LCD_DATA(0x35);
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0x02);
  //****************************************
  LCD_CMD(0xC5); // Set Frame Rate (VCOMCONTROL1)
  LCD_DATA(0x04); // 72Hz
  //****************************************
  LCD_CMD(0xD2); // Power Settings  (SETPWRNORMAL)
  LCD_DATA(0x01);
  LCD_DATA(0x44);
  //****************************************
  LCD_CMD(0xC8); //Set Gamma  (GAMMASET)
  LCD_DATA(0x04);
  LCD_DATA(0x67);
  LCD_DATA(0x35);
  LCD_DATA(0x04);
  LCD_DATA(0x08);
  LCD_DATA(0x06);
  LCD_DATA(0x24);
  LCD_DATA(0x01);
  LCD_DATA(0x37);
  LCD_DATA(0x40);
  LCD_DATA(0x03);
  LCD_DATA(0x10);
  LCD_DATA(0x08);
  LCD_DATA(0x80);
  LCD_DATA(0x00);
  //****************************************
  LCD_CMD(0x2A); // Set_column_address 320px (CASET)
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0x3F);
  //****************************************
  LCD_CMD(0x2B); // Set_page_address 480px (PASET)
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0xE0);
  //  LCD_DATA(0x8F);
  LCD_CMD(0x29); //display on
  LCD_CMD(0x2C); //display on

  LCD_CMD(ILI9341_INVOFF); //Invert Off
  delay(120);
  LCD_CMD(ILI9341_SLPOUT);    //Exit Sleep
  delay(120);
  LCD_CMD(ILI9341_DISPON);    //Display on
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para enviar comandos a la LCD - parámetro (comando)
//***************************************************************************************************************************************
void LCD_CMD(uint8_t cmd) {
  digitalWrite(LCD_RS, LOW);
  digitalWrite(LCD_WR, LOW);
  GPIO_PORTB_DATA_R = cmd;
  digitalWrite(LCD_WR, HIGH);
}
//***************************************************************************************************************************************
// Función para enviar datos a la LCD - parámetro (dato)
//***************************************************************************************************************************************
void LCD_DATA(uint8_t data) {
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_WR, LOW);
  GPIO_PORTB_DATA_R = data;
  digitalWrite(LCD_WR, HIGH);
}
//***************************************************************************************************************************************
// Función para definir rango de direcciones de memoria con las cuales se trabajara (se define una ventana)
//***************************************************************************************************************************************
void SetWindows(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) {
  LCD_CMD(0x2a); // Set_column_address 4 parameters
  LCD_DATA(x1 >> 8);
  LCD_DATA(x1);
  LCD_DATA(x2 >> 8);
  LCD_DATA(x2);
  LCD_CMD(0x2b); // Set_page_address 4 parameters
  LCD_DATA(y1 >> 8);
  LCD_DATA(y1);
  LCD_DATA(y2 >> 8);
  LCD_DATA(y2);
  LCD_CMD(0x2c); // Write_memory_start
}
//***************************************************************************************************************************************
// Función para borrar la pantalla - parámetros (color)
//***************************************************************************************************************************************
void LCD_Clear(unsigned int c) {
  unsigned int x, y;
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  SetWindows(0, 0, 319, 239); // 479, 319);
  for (x = 0; x < 320; x++)
    for (y = 0; y < 240; y++) {
      LCD_DATA(c >> 8);
      LCD_DATA(c);
    }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar una línea horizontal - parámetros ( coordenada x, cordenada y, longitud, color)
//***************************************************************************************************************************************
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c) {
  unsigned int i, j;
  LCD_CMD(0x02c); //write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  l = l + x;
  SetWindows(x, y, l, y);
  j = l;// * 2;
  for (i = 0; i < l; i++) {
    LCD_DATA(c >> 8);
    LCD_DATA(c);
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar una línea vertical - parámetros ( coordenada x, cordenada y, longitud, color)
//***************************************************************************************************************************************
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c) {
  unsigned int i, j;
  LCD_CMD(0x02c); //write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  l = l + y;
  SetWindows(x, y, x, l);
  j = l; //* 2;
  for (i = 1; i <= j; i++) {
    LCD_DATA(c >> 8);
    LCD_DATA(c);
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar un rectángulo - parámetros ( coordenada x, cordenada y, ancho, alto, color)
//***************************************************************************************************************************************
void Rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c) {
  H_line(x  , y  , w, c);
  H_line(x  , y + h, w, c);
  V_line(x  , y  , h, c);
  V_line(x + w, y  , h, c);
}
//***************************************************************************************************************************************
// Función para dibujar un rectángulo relleno - parámetros ( coordenada x, cordenada y, ancho, alto, color)
//***************************************************************************************************************************************
void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c) {
  unsigned int i;
  for (i = 0; i < h; i++) {
    H_line(x  , y  , w, c);
    H_line(x  , y + i, w, c);
  }
}
//***************************************************************************************************************************************
// Función para dibujar texto - parámetros ( texto, coordenada x, cordenada y, color, background)
//***************************************************************************************************************************************
void LCD_Print(String text, int x, int y, int fontSize, int color, int background) {
  int fontXSize ;
  int fontYSize ;

  if (fontSize == 1) {
    fontXSize = fontXSizeSmal ;
    fontYSize = fontYSizeSmal ;
  }
  if (fontSize == 2) {
    fontXSize = fontXSizeBig ;
    fontYSize = fontYSizeBig ;
  }

  char charInput ;
  int cLength = text.length();
  Serial.println(cLength, DEC);
  int charDec ;
  int c ;
  int charHex ;
  char char_array[cLength + 1];
  text.toCharArray(char_array, cLength + 1) ;
  for (int i = 0; i < cLength ; i++) {
    charInput = char_array[i];
    Serial.println(char_array[i]);
    charDec = int(charInput);
    digitalWrite(LCD_CS, LOW);
    SetWindows(x + (i * fontXSize), y, x + (i * fontXSize) + fontXSize - 1, y + fontYSize );
    long charHex1 ;
    for ( int n = 0 ; n < fontYSize ; n++ ) {
      if (fontSize == 1) {
        charHex1 = pgm_read_word_near(smallFont + ((charDec - 32) * fontYSize) + n);
      }
      if (fontSize == 2) {
        charHex1 = pgm_read_word_near(bigFont + ((charDec - 32) * fontYSize) + n);
      }
      for (int t = 1; t < fontXSize + 1 ; t++) {
        if (( charHex1 & (1 << (fontXSize - t))) > 0 ) {
          c = color ;
        } else {
          c = background ;
        }
        LCD_DATA(c >> 8);
        LCD_DATA(c);
      }
    }
    digitalWrite(LCD_CS, HIGH);
  }
}
//***************************************************************************************************************************************
// Función para dibujar una imagen a partir de un arreglo de colores (Bitmap) Formato (Color 16bit R 5bits G 6bits B 5bits)
//***************************************************************************************************************************************
void LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[]) {
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 = x + width;
  y2 = y + height;
  SetWindows(x, y, x2 - 1, y2 - 1);
  unsigned int k = 0;
  unsigned int i, j;

  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      LCD_DATA(bitmap[k]);
      LCD_DATA(bitmap[k + 1]);
      //LCD_DATA(bitmap[k]);
      k = k + 2;
    }
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar una imagen sprite - los parámetros columns = número de imagenes en el sprite, index = cual desplegar, flip = darle vuelta
//***************************************************************************************************************************************
void LCD_Sprite(int x, int y, int width, int height, unsigned char bitmap[], int columns, int index, char flip, char offset) {
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 =   x + width;
  y2 =    y + height;
  SetWindows(x, y, x2 - 1, y2 - 1);
  int k = 0;
  int ancho = ((width * columns));
  if (flip) {
    for (int j = 0; j < height; j++) {
      k = (j * (ancho) + index * width - 1 - offset) * 2;
      k = k + width * 2;
      for (int i = 0; i < width; i++) {
        LCD_DATA(bitmap[k]);
        LCD_DATA(bitmap[k + 1]);
        k = k - 2;
      }
    }
  } else {
    for (int j = 0; j < height; j++) {
      k = (j * (ancho) + index * width + 1 + offset) * 2;
      for (int i = 0; i < width; i++) {
        LCD_DATA(bitmap[k]);
        LCD_DATA(bitmap[k + 1]);
        k = k + 2;
      }
    }


  }
  digitalWrite(LCD_CS, HIGH);
}

//***************************************************************************************************************************************
// Función para ver si dos objetos chocaron
//***************************************************************************************************************************************
uint8_t collider(uint16_t objeto1[], uint16_t objeto2[], uint8_t d1[] , uint8_t d2[]) {
  uint16_t dx;
  uint16_t dy;
  uint16_t objeto_x = abs((objeto1[0] + d1[0]) - (objeto2[0] + d2[0]));
  uint16_t objeto_y = abs((objeto1[1] + d1[1]) - (objeto2[1] + d2[1]));
  dx = abs(d1[0] + d2[0]);
  dy = abs(d1[1] + d2[1]);
  if (((objeto_x - dx) < 0) && ((objeto_y - dy) < 0)) {
    return 1; //chocaron ambos ejes
  } else {
    return 0; //no choco
  }
}
//***************************************************************************************************************************************
// Función para mover a kirby
//***************************************************************************************************************************************
void kirby_spriter(uint8_t animacion, int anim, bool lado) {
  switch (animacion) {
    case 0: //parado
      if (lado == 1) {
        LCD_Sprite(kirby[0], kirby[1], 16, 16, kirby_standing, 1, 0, 1, 0);
      } else {
        LCD_Sprite(kirby[0], kirby[1], 16, 16, kirby_standing, 1, 0, 0, 0);
      }
      break;
    case 1: //correr
      if (lado == 1) {
        LCD_Sprite(kirby[0], kirby[1], 16, 16, kirby_running, 4, anim, 1, 15);
      } else {
        LCD_Sprite(kirby[0], kirby[1], 16, 16, kirby_running, 4, anim, 0, 15);
      }
      break;
    case 2: //saltar
      if (lado == 1) {
        LCD_Sprite(kirby[0], kirby[1], 16, 16, kirby_jumping, 1, 0, 1, 0);
      } else {
        LCD_Sprite(kirby[0], kirby[1], 16, 16, kirby_jumping, 1, 0, 0, 0);
      }
      break;
    case 3: //caer
      if (lado == 1) {
        LCD_Sprite(kirby[0], kirby[1], 16, 16, kirby_falling, 1, 0, 1, 0);
      } else {
        LCD_Sprite(kirby[0], kirby[1], 16, 16, kirby_falling, 1, 0, 0, 0);
      }
      break;
    case 4: //muere
      if (lado == 1) {
        LCD_Sprite(kirby[0], kirby[1], 16, 16, kirby_vanishing, 10, anim, 1, 0);
      } else {
        LCD_Sprite(kirby[0], kirby[1], 16, 16, kirby_falling, 10, anim, 0, 0);
      }
      break;
  }
}
//***************************************************************************************************************************************
// Función para mover a kirby
//***************************************************************************************************************************************
void kirby_spriter2(uint8_t animacion, int anim, bool lado) {
  switch (animacion) {
    case 0: //parado
      if (lado == 1) {
        LCD_Sprite(kirby2[0], kirby2[1], 16, 16, kirby_standing2, 1, 0, 1, 0);
      } else {
        LCD_Sprite(kirby2[0], kirby2[1], 16, 16, kirby_standing2, 1, 0, 0, 0);
      }
      break;
    case 1: //correr
      if (lado == 1) {
        LCD_Sprite(kirby2[0], kirby2[1], 16, 16, kirby_running2, 4, anim, 1, 15);
      } else {
        LCD_Sprite(kirby2[0], kirby2[1], 16, 16, kirby_running2, 4, anim, 0, 15);
      }
      break;
    case 2: //saltar
      if (lado == 1) {
        LCD_Sprite(kirby2[0], kirby2[1], 16, 16, kirby_jumping2, 1, 0, 1, 0);
      } else {
        LCD_Sprite(kirby2[0], kirby2[1], 16, 16, kirby_jumping2, 1, 0, 0, 0);
      }
      break;
    case 3: //caer
      if (lado == 1) {
        LCD_Sprite(kirby2[0], kirby2[1], 16, 16, kirby_falling2, 1, 0, 1, 0);
      } else {
        LCD_Sprite(kirby2[0], kirby2[1], 16, 16, kirby_falling2, 1, 0, 0, 0);
      }
      break;
    case 4: //muere
      if (lado == 1) {
        LCD_Sprite(kirby2[0], kirby2[1], 16, 16, kirby_vanishing, 10, anim, 1, 0);
      } else {
        LCD_Sprite(kirby2[0], kirby2[1], 16, 16, kirby_falling, 10, anim, 0, 0);
      }
      break;
  }
}
//***************************************************************************************************************************************
// Función para que kirby salte
//***************************************************************************************************************************************
void jumping(uint16_t personaje[], uint8_t numero, uint8_t *animacion, bool *jump_available, bool *move_y, uint16_t piso) {
  if (/*(kirby_anim==2)||(kirby_anim==3)||(kirby_anim==4)*/*jump_available == false) {
    if (personaje[1] == (last[numero] - 55)) {
      *move_y = down;
      *animacion = 3;
    }
    if (*move_y == up) {
      personaje[1] -= 1;
      H_line(personaje[0], personaje[1] + 16, 16, 0xffff);
    }
    else if (*move_y == down) {
      personaje[1] += 1;
      H_line(personaje[0], personaje[1] - 1, 16, 0xffff);
      if (personaje[1] == piso - 16) {
        *animacion = 1;
        *jump_available = true;
        *move_y = up;
      }
    }
  }
}
//***************************************************************************************************************************************
// Función para que saber de que manera chocó
//***************************************************************************************************************************************
uint8_t kirby_collision(uint8_t caso, uint8_t anim) {
  uint8_t last = anim;
  switch (caso) {
    case 0: //no choco
      return anim;
      break;
    case 1: //choco en ambos ejes
      return 4;
      break;
  }
}
uint8_t kirby2_collision(uint8_t caso, uint8_t anim) {
  uint8_t last = anim;
  switch (caso) {
    case 0: //no choco
      return anim;
      break;
    case 1: //choco en ambos ejes
      return 4;
      break;
  }
}
//***************************************************************************************************************************************
// Función que convierte de char con signo a char sin signo
//***************************************************************************************************************************************
unsigned char Char_uChar(char value){
  unsigned char entero;
  if(value>=48 && value <=57){
    entero = value - 48;
  }
  else if (value >= 97 && value<=102){
    entero = value -87;
  }
  return entero;
}
//***************************************************************************************************************************************
// Función que lee y despliega de la SD
//***************************************************************************************************************************************
void SD_Bitmap(int x, int y, int width, int height, char *filename){
  uint16_t SD_x;
  uint16_t SD_y;
  uint16_t ent;
  uint16_t contador = 1;
  uint16_t temp;
  uint16_t temp2;
  uint16_t linea_salto = 1;
  myFile = SD.open(filename);
  LCD_CMD(0x02c); //write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  SetWindows(x,y,x+width-1,y+height-1);
  if (myFile){
    SD_x = x;
    SD_y = y;
    while (myFile.available()) {
      uint16_t leer = myFile.read();
      leer = Char_uChar(leer);
        if (contador == 1){
          temp2 = leer << 12;
          temp = temp2;
          contador++;
        }
        else if (contador == 2){
          temp2 = leer << 8;
          temp = temp+temp2;
          contador++;
        }
        else if (contador == 3){
          temp2 = leer<<4;
          temp = temp+temp2;
          contador++;
        }
        else if (contador == 4){
          linea_salto++;
          contador = 1;
          temp = temp+leer;
          LCD_DATA(temp>>8);
          LCD_DATA(temp);
          SD_x = SD_x+1;
          if(SD_x==((x+width))){
            SD_x = x;
            SD_y = SD_y+1;
          }
        }
        
          else;
        }
      
      digitalWrite(LCD_CS, HIGH);
      myFile.close();
  }
  
  else{
    Serial.print("error opening ");
  }
}
