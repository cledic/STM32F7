/*
  01 Set 1992 Inserita gestione menu'
  12 Apr 1992 SpinLine ed Extrude.
  04 Dic 1991 Adattato al Borland C++. Versione di test per lo sviluppo.
  17 Mar 1991 Inserito il release dei shape in uscita. Sembra non dar
			  problemi.
  15 Mar 1991 Implementazione del sort dei shape. Cambiato il nome
			  dei file da .BIN in .BIP
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "3d.h"


/* Procedure esterne a questo modulo */
extern void start_3d(void);
extern void draw_3d(int);
extern int read_points(void);

/*
  Puntatore di inizio lista.
*/
TRIANGLE *MARKER;

COORD *pos[MAX_VERT];
/*
  Valori x ed y nel dominio 2D
*/
int  *px2d[MAX_POINT];
int  *py2d[MAX_POINT];
int  *color2d[MAX_POINT];
/*
  Numero di vertici dell'oggetto.
*/
int  vertices;
/*
  Numero di triangoli che costituiscono l'oggetto.
*/
int  shape;
/*
  Variabili di configurazione del programma.
  snap  0 	-> 	no snap
		1 	->	snap sulla griglia *

  mark	SQR	->	mark quadrato *
		CIR	->	mark rotondo

  mode_grid	NORMAL	-\ *
			SMALL   -   grandezza griglia
			BIG     -/

  spinline  0	->	Extrude
			1	->	SpinLine *

  num_rot	32	->  Numero di rotazioni in SpinLine *

  Quelle segnate con * sono di default.
*/

/* SNAP sulla griglia */
#if 1
int snap = TRUE;
#else
int snap = FALSE;
#endif

/* Tipo mark (quadrato circolare) */
#if 1
int mark = SQR;
#else
int mark = CIR;
#endif

/* Grandezza griglia: NORMAL, SMALL, BIG */
int mode_grid = BIG; 

/* Modalità spinline o extrude */
#if 1
int spinline = TRUE;
#else
int spinline = FALSE;
#endif

/* Numero di rotazioni per lo SpinLine */
int num_rot = 32;

int mn_start, mn_rot, mn_zoom;

int Init3D( void)
{
  init_3d();                  /* Inizializza la lista linkata doppia ad anello.!!! */
  
  if (read_points()==-1)      /* Procedura per la creazione dell'obj. */
   return( 1);                /* Per uscire dal loop di acquisizione, fare tap sull'angolo in basso a DX!! */
  
  start_3d();                 /* Applica all'oggetto una prima rotazione. */
  draw_3d(PLOT);              /* Disegna l'oggetto. */
    
  return( 0);
}

/*
  Inizializza la lista linkata doppia ad anello.!!!
*/
void init_3d( void)
{
 MARKER = ( TRIANGLE *)malloc(sizeof( TRIANGLE));
 MARKER->previous = MARKER->next = MARKER;
}
/*
  Ritorna di un triangolo la coordinata Z piu' piccola.
*/
float Min_Z( TRIANGLE *a)
{
 float u, d;

 u = Min(pos[a->first]->z, pos[a->second]->z);
 d = Min(u, pos[a->third]->z);
 return(d);
}
/*
  Ritorna di due float il valore piu' piccolo.
*/
float Min( float a, float b)
{
 if (a==b)
	return(a);

 if (a<b)
	return(a);
 else
	return(b);
}
/*
  Inserisce una struttura nella lista.
  restituisce  0 -> sempre
			  -1 -> in caso di errore nella lista.
*/
int insert( TRIANGLE *before, TRIANGLE *after, TRIANGLE *current)
{
 if (before->next != after)
	 return(-1);
 if (before != after->previous)
	 return(-1);

 before->next = current;
 current->previous = before;
 after->previous = current;
 current->next = after;
 /* Incrementa il contatore dei triangoli */
 shape++;
 return(0);
}

/*
  Ritorna il puntatore all struttura seguente a quella passatagli.
*/
TRIANGLE *sequence( TRIANGLE *punt)
{
 TRIANGLE *value;

 if (punt == 0)
	 punt = MARKER;

 if ((value = punt->next) == MARKER)
	return((TRIANGLE *)NULL);
 else
	return(value);
}

/*
  Ritorna 0 se i due triangoli hanno coord. Z uguali
		 -1 se v e' minore di n
		  1 se v e' maggiore di n
*/
int evaluate( TRIANGLE *v, TRIANGLE *n)
{
 float c, d;
 int r=0;

 c = Min_Z(v);
 d = Min_Z(n);

 if (c==d)
	return(0);
 else
	if (c<d)
	   return(-1);
	else
		 return(1);
 
}

/*
  Elimina un elemento dalla struttura. Non lo cancella, ma mette NULL
  nei due link alla successiva e alla precedente.
*/
int cut( TRIANGLE *val)
{
 TRIANGLE *before, *after;

 before = val->previous;
 after = val->next;

 before->next = after;
 after->previous = before;
 val->previous = val->next = ( TRIANGLE *)NULL;

 return(0);
}
/*
  Scambia di posizione a due strutture dati.
  ritorna 0 -> Sempre.
		 -1 -> In caso di errore.
*/
int swap( TRIANGLE *f, TRIANGLE *s)
{
 if (cut(s))
	return(-1);
 return( insert(f->previous, f, s) );
}
/*
  Implementazione di un Bubble sort. Le funzioni sono
  tutte predefinite al contrario dei sort standard che vedono
  le procedure di compare, swap fornite dall'utente.
  Ritorna 0 -> Sempre.
	 -1 -> In caso di errore.
*/
int sort()
{
 int flag;
 TRIANGLE *p1, *p2;

 do
  {
   flag = 0;
   p2 = ( TRIANGLE *)sequence(( TRIANGLE *)NULL);
   /*p1 = p2;*/
   while(p1=p2,(p2 = ( TRIANGLE *)sequence(p2)) != /*(struct triangle *)*/NULL)
		{
		 if (evaluate(p1, p2) < 0)
			 {
				if (swap(p1, p2))
				 return(-1);
				flag = 1;
			 }
		}
  }while(flag);

 return(0);
}
/*
*/
void release( void)
{
 TRIANGLE *value, *old;

 value = MARKER->next;
 while(value != MARKER)
  {
   old = value;
   value = value->next;
   free(old);
  }
}

void doSpin(void)
{
 spinline=TRUE;
}
void doExtrude(void)
{
 spinline=FALSE;
}