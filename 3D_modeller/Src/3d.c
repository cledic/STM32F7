/*
  Modulo tratto dal Moudla 2 FTL per la costruzione di un oggetto
  in 3 dimensioni che ruota

  19 Feb 91 Modificato per interpretare tutti e tre i punti messi a
			disposizione da Stereo CAD
  28 Feb 91 Inserita gestione lista linkata semplice dei triangoli.
  15 Mar 91 Modificata la lista in linkata doppia. Inserito il sort.
  17 Mar 91 Ok e' funzionante.
  19 Mar 91 Implementata la visualizzazione prospettica.
  04 Dic 91 Adattato al Borland C++. Versione di test per lo sviluppo.
  23 Feb 92 Ho implementato l'uso delle matrici per la rotazione sui
			tre assi.
  29 Feb 92 Il programma gira bene meno il sort delle superfici. Comunque
			faccio troppo uso delle variabili globali.
  29 Mar 92 Ho inserito anche la matrice di traslazione e quella di
			trasformazione in scala.
*/

#include <stdio.h>
#include <math.h>
#include "arm_math.h"
#include "arm_const_structs.h"
#include "3d.h"

extern unsigned int doRedraw;
extern COORD *pos[];
extern TRIANGLE *MARKER;
extern int *px2d[], *py2d[];
extern int  *color2d[MAX_POINT];
extern int shape;
extern int vertices;
extern int sort(void);
extern TRIANGLE *sequence( TRIANGLE *punt);
extern TIM_HandleTypeDef     TimHandle;
extern uint32_t time_start, time_end, time_diff;
extern float tdiff[5];

#define TIM_MEASURE_START {time_start = __HAL_TIM_GET_COUNTER(&TimHandle);}

#define TIM_MEASURE_END(x) {time_end = __HAL_TIM_GET_COUNTER(&TimHandle);     \
                        time_diff = time_end - time_start;								\
												tdiff[x]=(float)time_diff*(float)0.000000005;}

/* */
typedef float Transform3D[4][4];
Transform3D r1, r2, xform;
float  distance=40.0;
float  zc=50.0, xc=50.0, yc=50.0;

/*
  Dichiarazione dei prototipi.
*/
void DrawShapeWireFrame( int m);
void ParallelTransform( void);
void start_3d( void);
void draw_3d( int);
void XYrot( int);
void Zoom( int);
void Zoom( int);
void Mv( int);
void show_pos( int a, int b, int c, int d);
void show_info( void);
void esegui_mv( int a, int b);
void esegui_tr( int a);
/* */
void XYZTrasl( int a);
void ConcatMat( void);
void PartialNonHomTransform( void);
void FormatRotateMat(char axis, int dir);
void FormatTranslMat( char axis, float val);
void IdentMat( void);
void IdentMat2( void);
void DoMove( void);
void SwapMat( void);

/* Assembler function */
void PartialNonHomTransform_asm( COORD**pPos, Transform3D*pXForm, unsigned int size);
void ParallelTransform_asm( int**px2d, int**py2d, COORD**pPos, unsigned int size);

#define XMove( dir)       FormatTranslMat('x', (float)2.0f*dir)
#define XRotation( dir)   FormatRotateMat('x', dir)
#define YRotation( dir)   FormatRotateMat('y', dir)
#define ZRotation( dir)   FormatRotateMat('z', dir)
#define YMove( dir)       FormatTranslMat('y', (float)2.0f*dir)
#define ZMove( dir)       FormatTranslMat('z', (float)2.0f*dir)

typedef struct _VERTEX {
  int x;
  int y;
} VERTEX;

/* Array per meorizzare le coordinate dei segmenti. */
int32_t seg1_x[400], seg2_x[400], seg3_x[400];
int32_t seg1_y[400], seg2_y[400], seg3_y[400];
int32_t tmp_x[400], tmp_y[400];

int mycolor[] = {0x00FFFFFF, 0x007F7F7F, 0x003F3F3F, 0x001F1F1F, 0x000F0F0F, 0x00070707, 0x00030303, 0x00010101};
VERTEX A, B, C, tmp;

uint8_t text[100];                                   /* File read buffer */

/*
  Procedura per la visualizzazione wire frame dell'oggetto 3D
*/
void DrawShapeWireFrame( int m)
{
  TRIANGLE *value;
  value = MARKER;
	
	BSP_LCD_SetTextColor( m);
	//
  while((value = sequence(value)) != ( TRIANGLE *)NULL)
	{   
			BSP_LCD_DrawLine( *px2d[value->first], *py2d[value->first], *px2d[value->second], *py2d[value->second]);
			BSP_LCD_DrawLine( *px2d[value->second], *py2d[value->second], *px2d[value->third], *py2d[value->third]);
			BSP_LCD_DrawLine( *px2d[value->third], *py2d[value->third], *px2d[value->first], *py2d[value->first]);  			
	}
}

/*
  Procedura per la conversione delle coordinate da 3 a 2 dimensioni
  utilizzando la proiezione parallela.
*/
void ParallelTransform( void)
{
#if 0
 int i=0;
 int addingX, addingY;
 float f;

 addingX=getmaxx()/2;
 addingY=getmaxy()/2;

	TIM_MEASURE_START;
  while(i<vertices)
	{
		f = (float)(1000.0f / (distance - pos[i]->z));
		*px2d[i] = (int) (pos[i]->x*f) + addingX;
		*py2d[i] = (int) (pos[i]->y*f) + addingY;
		*color2d[i] = (int)f;
		i++;
	}
	TIM_MEASURE_END(3);
#else
	TIM_MEASURE_START;
	ParallelTransform_asm( (int**)px2d, (int**)py2d, (COORD**)pos, vertices);
	TIM_MEASURE_END(3);
#endif	
}

/*
*/
void start_3d( void)
{
  XYrot(3);
}
/*

*/
void esegui_mv( int c,  int k)
{

  /* Per cancellare l'oggetto dalla vecchia posizione pulisco un rettangolo di schermo. */
	BSP_LCD_Clear(UNPLOT);
  
	if (k==4)    /* */
    XYrot(c);
  else if (k==8) /* */
          Zoom(c);
  else
     Mv(c);
  /* */
	show_info();
  draw_3d(PLOT);
}
/*
   Disegnare l'oggetto in 3D.
*/
void draw_3d( int w)
{
  ParallelTransform();
  DrawShapeWireFrame( w);
}
/*
*/
void XYrot( int i)
{
 switch(i) {
   case  1: XRotation(1); IdentMat2(); break;                 /* Est */
   case  2: YRotation(1); IdentMat2(); break;                 /* Sud */
   case  3: XRotation(1); SwapMat(); YRotation(1); break;     /* SE */
   case  4: XRotation(-1); IdentMat2(); break;                /* Ovest */
   case  6: XRotation(-1); SwapMat(); YRotation(1); break;    /* SO */
   case  7: YRotation(-1); IdentMat2(); break;                /* Nord */
   case  8: XRotation(1); SwapMat(); YRotation(-1); break;    /* NE */
   case 11: XRotation(-1); SwapMat(); YRotation(-1); break;   /* NO */
 }
 ConcatMat();
 PartialNonHomTransform();
}
/*
*/
void Zoom( int i)
{
 switch(i) {
  case 2:
  case 3:
  case 6: distance += 1.0f;break;
  case 7:
  case 8:
  case 11: distance -= 1.0f;break;
 }
}

/*
  Procedura per il movimentu sugli assi.
*/
void Mv( int i)
{
 switch(i) {
   case  cMOVE_EST:   XMove(1); IdentMat2(); break;             /* Est */
   case  cMOVE_NORD:  YMove(-1); IdentMat2(); break;            /* Nord */
   case  cMOVE_NE:    XMove(1); SwapMat(); YMove(-1); break;    /* NE */
   case  cMOVE_OVEST: XMove(-1); IdentMat2(); break;            /* Ovest */
   case  cMOVE_NO:    XMove(-1); SwapMat(); YMove(-1); break;   /* NO */
   case  cMOVE_SUD:   YMove(1); IdentMat2(); break;             /* Sud */
   case  cMOVE_SE:    XMove(1); SwapMat(); YMove(1); break;     /* SE */
   case  cMOVE_SO:    XMove(-1); SwapMat(); YMove(1); break;    /* SO */
 }
 ConcatMat();
 DoMove();
}

/*
*/
void show_info( void)
{
	BSP_LCD_SetTextColor( LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font12);
  sprintf((char*)text, "Vertices #%4d Shapes #%4d", vertices, shape);
	BSP_LCD_DisplayStringAtLine( 0, text);
}

/******************************************************************************
 * Description
 *	Concatenate two 4-by-4 transformation matrices.
 *
 * Input
 *	l		multiplicand (left operand)
 *	r		multiplier (right operand)
 *
 * Output
 *	*m		Result matrix
 *****************************************************************************/

void ConcatMat( void)
	{
	int i;
	int j;

	for (i = 0; i < 4; ++i)
		for (j = 0; j < 4; ++j)
			xform[i][j] = r1[i][0] * r2[0][j]
				+ r1[i][1] * r2[1][j]
				+ r1[i][2] * r2[2][j]
				+ r1[i][3] * r2[3][j];
	}



/******************************************************************************
 * Description
 *	Format a matrix that will perform a rotation transformation
 *	about the specified axis.  The rotation angle is measured
 *	counterclockwise about the specified axis when looking
 *	at the origin from the positive axis.
 *
 * Input
 *	axis		Axis ('x', 'y', 'z') about which to perform rotation
 *	r1		    Is a pointer to rotation matrix
 *
 * Output
 *****************************************************************************/

void FormatRotateMat(char axis, int dir)
	{
	float s, c;

	IdentMat();

	s = sinphi;
	c = cosphi;

	switch(axis)
		{
		case 'x':
			r1[1][1] = r1[2][2] = c;
			r1[1][2] = (float)(s*dir);
			r1[2][1] = (float)(-s*dir);
			break;
		case 'y':
			r1[0][0] = r1[2][2] = c;
			r1[2][0] = (float)(s*dir);
			r1[0][2] = (float)(-s*dir);
			break;
		case 'z':
			r1[0][0] = r1[1][1] = c;
			r1[0][1] = (float)(s*dir);
			r1[1][0] = (float)(-s*dir);
			break;
		}
	}

/******************************************************************************
 * Description
 *	Format a matrix that will perform a translation
 *	about the specified axis.
 *
 * Input
 *	axis		Axis ('x', 'y', 'z') about which to perform rotation
 *	r1		    Is a pointer to rotation matrix
 *
 * Output
 *****************************************************************************/
void FormatTranslMat( char axis, float val)
{
  IdentMat();
  switch(axis)
	{
	 case 'x':
		r1[2][0] = val;
		r1[2][1] = r1[2][2] = 0;
		break;
	 case 'y':
		r1[2][1] = val;
		r1[2][0] = r1[2][2] = 0;
		break;
	 case 'z':
		r1[2][2] = val;
		r1[2][0] = r1[2][1] = 0;
		break;
	 }
}


/******************************************************************************
 * Description
 *	Format a matrix that will perform a scale transformation
 *	about the specified axis.
 *
 * Input
 *	axis		Axis ('x', 'y', 'z') about which to perform transformation
 *	val		    percentage
 *
 * Output
 *****************************************************************************/
void FormatScaleMat(char axis, float val)
{
  IdentMat();
  switch(axis)
	{
	 case 'x':
		r1[0][0] = val;
		r1[1][1] = r1[2][2] = 1;
		break;
	 case 'y':
		r1[1][1] = val;
		r1[0][0] = r1[2][2] = 1;
		break;
	 case 'z':
		r1[2][2] = val;
		r1[0][0] = r1[1][1] = 1;
		break;
	 }
}

/******************************************************************************
 * Description
 *	Format a 4x4 identity matrix.
 *
 * Output
 *	*m		Formatted identity matrix
 *****************************************************************************/

void IdentMat( void)
{
 int i;
 int j;

 for (i = 3; i >= 0; --i)
	{
	for (j = 3; j >= 0; --j)
		r1[i][j] = 0.0;
	r1[i][i] = 1.0;
	}
}

void IdentMat2( void)
{
 int i;
 int j;

 for (i = 3; i >= 0; --i)
	{
	for (j = 3; j >= 0; --j)
		r2[i][j] = 0.0;
	r2[i][i] = 1.0;
	}
}



/******************************************************************************
 * Description
 *	Perform a partial transform on non-homogeneous points.
 *	Given an array of non-homogeneous (3-coordinate) input points,
 *	this routine multiplies them by the 3-by-3 upper left submatrix
 *	of a standard 4-by-4 transform matrix.  The resulting non-homogeneous
 *	points are returned.
 *
 * Input
 *	m		4-by-4 transform matrix
 *
 *****************************************************************************/

void PartialNonHomTransform( void)
{
#if 0
 int i=0;
 float tmp_x, tmp_y, tmp_z;

	TIM_MEASURE_START;
 while(i<vertices)
 {
  tmp_x = pos[i]->x * xform[0][0] + pos[i]->y * xform[1][0] + pos[i]->z * xform[2][0];
  tmp_y = pos[i]->x * xform[0][1] + pos[i]->y * xform[1][1] + pos[i]->z * xform[2][1];
  tmp_z = pos[i]->x * xform[0][2] + pos[i]->y * xform[1][2] + pos[i]->z * xform[2][2];
  pos[i]->x=tmp_x;
  pos[i]->y=tmp_y;
  pos[i]->z=tmp_z;
  i++;
 }
 TIM_MEASURE_END(2);
#else
 TIM_MEASURE_START;
 PartialNonHomTransform_asm( (COORD**)pos, (Transform3D*)xform, vertices);
 TIM_MEASURE_END(2);
#endif 
}
/*

*/
void DoMove( void)
{
 int i=0;

 while(i<vertices)
 {
  pos[i]->x+=xform[2][0];
  pos[i]->y+=xform[2][1];
  pos[i]->z+=xform[2][2];
  i++;
 }
}

/*
  Copia il contenuto della matrice r1 nella matrice r2
*/
void SwapMat( void)
{
 int i;
 int j;

	for (i = 3; i >= 0; --i)
		{
		for (j = 3; j >= 0; --j)
			r2[i][j] = r1[i][j];
		}
}

/* Campo di prova per la trasformazione in scala */
/*
*/
void XTrasl(void )
{
 FormatScaleMat('x', (float)1.1f);
}
/*
*/
void YTrasl( void)
{
 FormatScaleMat('y', (float)1.1f);
}
/*
*/
void ZTrasl( void)
{
 FormatScaleMat('z', (float)1.1f);
}
/*
*/
void XYZTrasl( int i)
{
 switch(i) {
   case  'X': XTrasl(); IdentMat2(); break;
   case  'Y': YTrasl(); IdentMat2(); break;
   case  'Z': ZTrasl(); IdentMat2(); break;
 }
 ConcatMat();
 PartialNonHomTransform();
}
/*
*/
void esegui_tr( int c)
{
 if (c=='X' || c=='Y' || c=='Z')
  {
   BSP_LCD_Clear(UNPLOT);
   XYZTrasl(c);
   draw_3d(PLOT);
  }
}
