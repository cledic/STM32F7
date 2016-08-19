
	AREA     ||.text||, CODE, READONLY, ALIGN=2
	

;/*
;* @brief  Receive the TS_State point array and return the array of distance between points.
;* @param[in]  *pTchX    [r0]    points to the array of short X coord.
;* @param[in]  *pTchY    [r1]    points to the array of short Y coord.
;* @param[out] *pDstn    [r2] 	points to the array of float32_t distance calculated
;* @param[in]  size      [r3]	number of points.
;* @return none.
;*
;* void DistanceCalc_asm( short*pTchX, short*pTchY, float*pDstn, unsigned int size);
;*
;*/
	EXPORT DistanceCalc_asm

DistanceCalc_asm PROC
	PUSH     {r4-r11}
	CMP		 r0, #0				; check if pointer pThcX is NULL
	BEQ		 DistanceCalc_asm_exit
	MOV		 r4, r0				; load the pSrc pointer	
	
	CMP		 r1, #0				; check if pointer pTchY is NULL
	BEQ		 DistanceCalc_asm_exit
	MOV		 r5, r1				; load the pDst pointer
	
	CMP		 r2, #0				; check if pointer pDstn is NULL
	BEQ		 DistanceCalc_asm_exit
	MOV		 r6, r2				; load the pDst pointer
	
	CMP		 r3, #0				; check if counter is zero
	BEQ		 DistanceCalc_asm_exit
	MOV		 r7, r3				; load array size
	SUBS	 r7, r7, #1			; decrement the counter
	CMP		 r7, #0				; check if counter is zero
	BEQ		 DistanceCalc_asm_exit
	
DistanceCalc_asm_0
;   // Copio la prima coppia di coordinate x1,y1 nei registri della FPU
	LDRSH	 r8, [r4], #2		; load from the X array x1 into r8
	LDRSH	 r9, [r5], #2		; load from the Y array y1 into r9
	vmov 	 s0,r8          	; copy the contents of r8 to s0
	vmov 	 s1,r9          	; copy the contents of r9 to s1
;   // Copio la seconda coppia di coordinate x2,y2 nei registri della FPU
	LDRSH	 r8, [r4]    		; load from the X array x2 into r8
	LDRSH	 r9, [r5]    		; load from the Y array y2 into r9
	vmov 	 s2,r8          	; copy the contents of r8 to s2
	vmov 	 s3,r9          	; copy the contents of r9 to s3
	
	vcvt.f32.s32 s0,s0  		; convert s0,x1 from signed int to float
	vcvt.f32.s32 s1,s1  		; convert s1,y1 from signed int to float
	vcvt.f32.s32 s2,s2  		; convert s2,x2 from signed int to float
	vcvt.f32.s32 s3,s3  		; convert s3,y2 from signed int to float
	
	vsub.f32 s4,s0,s2			; s4=s0,x1-s2,x2
	vmul.f32 s4,s4,s4			; s4=s4*s4

	vsub.f32 s5,s1,s3			; s5=s0,y1-s2,y2
	vmul.f32 s5,s5,s5			; s5=s5*s5
	
	vadd.f32 s6,s4,s5			; s6=s4+s5 
	
	vsqrt.f32 s6,s6				; s6=sqrt(s6)
	
	vmov	  r8,s6				;
	
	STR 	 r8, [r6], #4		; store the distance value to the pDstn

	SUBS	 r7, r7, #1			; decrement the counter
	BNE		 DistanceCalc_asm_0
	B DistanceCalc_asm_exit

DistanceCalc_asm_exit
	POP      {r4-r11}
	BX       lr
	ENDP
	
	END

