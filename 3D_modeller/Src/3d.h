/*
  File include per i programmi 3D
*/
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_sd.h"
#include "stm32f7xx_hal_i2s.h"
#include "stm32f7xx_hal_lptim.h"
#include "stm32f7xx_hal_tim.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_audio.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"

/*
 Definizioni per la scrittura
*/
#define DRAW_ON  1
#define DRAW_OFF 0

/*
  Define per la griglia
*/
#define HALF_WIDTH 200     /* Grandezza della griglia: 400x250 */
#define HALF_HIGT  125

/* Tipi di griglia */
#define SMALL  1           /* Tipi di griglia programmabili */
#define NORMAL 2
#define BIG    3

/* Distanza mouse-bordo_campo */
#define MOUSE_LIMIT 5

/* Define per il marker. */
#define MARK_W 5           // Grandezza del mark.
#define SQR 1              // Codice per il mark quadrato
#define CIR 2              // Codice per il mark a cerchio

/*
  Define per uso generale dal modulo 3DAPP
*/
#define START 1
#define END 0
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
#define MAX_POINT 1024
#define MAX_VERT  1024
#define CLIP_ON 1
#define Y_OFF 1
#define BACKSPACE 8
/*
  Define dal modulo 3D
*/
#define PLOT    LCD_COLOR_WHITE
#define UNPLOT  LCD_COLOR_BLACK
#define points  6
/* 360 gradi diviso 32 rotazioni = 11,25 */
#define sinphi  0.195090322f        /* sin( 11,25) = 0.195090322 */
#define cosphi  0.980785280f        /* cos( 11,25) = 0.980785280 */
/*
  Define per il modulo PARSER
*/
#define SNAP "-snap"
#define MARK "-mark"
#define GRID "-grid"
#define EXTR "-extr"
#define ROT  "-rot"
/* */
#define cEXEC_XYROT   4
#define cEXEX_ZOOM    8
#define cEXEC_MOVE    1
/* */
#define cZOOM_IN    6
#define cZOOM_OUT   11
/* */
#if 0
	#define cMOVE_EST     1
	#define cMOVE_NORD    7 //2
	#define cMOVE_NE      (cMOVE_NORD+cMOVE_EST) //3
	#define cMOVE_OVEST   4
	#define cMOVE_NO      (cMOVE_NORD+cMOVE_OVEST) //6
	#define cMOVE_SUD     2 //7
	#define cMOVE_SE      (cMOVE_SUD+cMOVE_EST) //8
	#define cMOVE_SO      (cMOVE_SUD+cMOVE_OVEST) //11
	/* */
	#define cXY_EST       (cMOVE_EST)
	#define cXY_SUD       (cMOVE_SUD)
	#define cXY_SE        (cMOVE_SE)
	#define cXY_OVEST     (cMOVE_OVEST)
	#define cXY_SO        (cMOVE_SO)
	#define cXY_NORD      (cMOVE_NORD)
	#define cXY_NE        (cMOVE_NE)
	#define cXY_NO        (cMOVE_NO)
#else
	#define cMOVE_EST     1
	#define cMOVE_NORD    7 //2
	#define cMOVE_NE      (cMOVE_NORD+cMOVE_EST) //3
	#define cMOVE_OVEST   4
	#define cMOVE_NO      (cMOVE_NORD+cMOVE_OVEST) //6
	#define cMOVE_SUD     2 //7
	#define cMOVE_SE      (cMOVE_SUD+cMOVE_EST) //8
	#define cMOVE_SO      (cMOVE_SUD+cMOVE_OVEST) //11
	/* */
	#define cXY_EST       (cMOVE_EST)
	#define cXY_SUD       (cMOVE_SUD)
	#define cXY_SE        (cMOVE_SUD)
	#define cXY_OVEST     (cMOVE_OVEST)
	#define cXY_SO        (cMOVE_SUD)
	#define cXY_NORD      (cMOVE_NORD)
	#define cXY_NE        (cMOVE_NORD)
	#define cXY_NO        (cMOVE_NORD)
#endif
/*
*/
#define getmaxx() 480
#define getmaxy() 272

/*
  Struttura per la memorizzazione delle coordinate
  di un singolo vertice.
*/
typedef struct _COORD {
	   float x, y, z;
} COORD;

/*
  Lista linkata ad anello per la memorizzazione dei triangoli
  generati dai vertici.
*/
typedef struct _TRIANGLE {
		  int first, second, third;
		  int cmd;                  
		  struct _TRIANGLE *next, *previous;
} TRIANGLE;

int insert(TRIANGLE *before, TRIANGLE *after, TRIANGLE *current);
/* Definizione dei prototipi */
int analizza( int x, int y, int b, int k, int d, int e);
int read_shape(void);
void init(void);
float Min_Z( TRIANGLE *a);
float Min(float a, float b);
// int insert(TRIANGLE *before, TRIANGLE *after, TRIANGLE *current);
TRIANGLE *sequence(TRIANGLE *punt);
int cut(TRIANGLE *val);
int swap(TRIANGLE *f, TRIANGLE *s);
int sort(void);
void release(void);
void show_sort(int a);
#if 0
int multi(void);
#endif
void uso(void);
void doSpin(void);
void doExtrude(void);
void init_3d( void);
int Init3D( void);
int evaluate( TRIANGLE *v, TRIANGLE *n);
