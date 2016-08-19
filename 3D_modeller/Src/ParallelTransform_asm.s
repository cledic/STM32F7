
	AREA     ||.text||, CODE, READONLY, ALIGN=2
	
	EXTERN distance
	EXTERN color2d
	
;/*
;* @brief  Receive the TS_State point array and return the array of distance between points.
;* @param[out]  **px2d   [r0]    points to pointers of int
;* @param[out]  **py2d   [r1]    points to pointers of int
;* @param[in]  **pPos   [r2]	pointer to pointers of floats
;* @param[in]  size     [r3]	number of points.
;* @return none.
;*
;* void ParallelTransform_asm( int**px2d, int**py2d, COORD**pPos, unsigned int size);
;*
;*/
	EXPORT ParallelTransform_asm

ParallelTransform_asm PROC
	PUSH     {r4-r12}
	CMP		 r0, #0				; check if pointer px2d is NULL
	BEQ		 ParallelTransform_asm_exit
	MOV		 r4, r0				; load the pSrc pointer	
	
	CMP		 r1, #0				; check if pointer py2d is NULL
	BEQ		 ParallelTransform_asm_exit
	MOV		 r5, r1				; load the pDst pointer

	CMP		 r2, #0				; check if pointer pPos is NULL
	BEQ		 ParallelTransform_asm_exit
	MOV		 r6, r2				; load the pDst pointer

	CMP		 r3, #0				; check if size is NULL
	BEQ		 ParallelTransform_asm_exit
 	MOV		 r7, r3				; load array size

; Load constant values
	vldr.f32 s10, =240.0		; load s10 with 240.0 s10=240.0 addingx
	vldr.f32 s11, =136.0		; load s11 with 136.0 s11=136.0 addingy
	
	LDR		 r8, =distance		;	
	vldr	 s12,[r8]			; copy the contents of r8  to s12   s12=distance
	
	LDR		 r12, =color2d		;	

	vldr.f32 s13, =1000.0		; load s13 with 1000.0 s13=1000.0

; Start loop	
ParallelTransform_asm_0
	LDR		 r8, [r6], #4
;   // 
	vldr	 s0, [r8]			; load x into s0
	vldr	 s1, [r8, #4]		; load y into s1
	vldr	 s2, [r8, #8]		; load z into s2
	
;   s20 =       s13 / ( s12 - s2)  	
;	f = (float)(1000.0f / (distance - pos[i]->z));
    vsub.f32 s23,s12,s2			; s12=s12-s2
	vdiv.f32 s20, s13,s23		; s20=s13/s12
	
;    s21 =             (s0*s20) + s10
;	*px2d[i] = (int) (pos[i]->x*f) + addingX;
	vmul.f32 s21,s0,s20
	vadd.f32 s21,s21,s10	 
	
;    s22 =             (s1*s20) + s11
;	*py2d[i] = (int) (pos[i]->y*f) + addingY;
	vmul.f32 s22,s1,s20
	vadd.f32 s22,s22,s11
	
	vcvt.s32.f32 s20,s20  		; convert s20 from float to signed int
	vcvt.s32.f32 s21,s21  		; convert s21 from float to signed int 
	vcvt.s32.f32 s22,s22  		; convert s22 from float to signed int 
	
	LDR		 r8, [r4], #4		; read the pointer from pxd2
	vstr	 s21, [r8]			; store the distance value to the *px2d[r7]
	LDR		 r8, [r5], #4		; read the pointer from pyd2
	vstr	 s22, [r8]

	LDR		 r8,[r12], #4		; read **color
	vstr	 s20,[r8]			; store s20 to **color

 	SUBS	 r7, r7, #1			; decrement the counter
 	BNE		 ParallelTransform_asm_0
	B ParallelTransform_asm_exit

ParallelTransform_asm_exit
	POP      {r4-r12}
	BX       lr
	ENDP
	
	END

