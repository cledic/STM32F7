
	AREA     ||.text||, CODE, READONLY, ALIGN=2
	

;/*
;* @brief  Receive the TS_State point array and return the array of distance between points.
;* @param[in]  **pPos   [r0]    pointer to pointers of float
;* @param[in]  *pXForm  [r1]    points to the array of float
;* @param[in]  size     [r2]	number of points.
;* @return none.
;*
;* void PartialNonHomTransform_asm( float**pPos, float*pXForm, unsigned int size);
;*
;*/
	EXPORT PartialNonHomTransform_asm

PartialNonHomTransform_asm PROC
	PUSH     {r4-r11}
	CMP		 r0, #0				; check if pointer pThcX is NULL
	BEQ		 PartialNonHomTransform_asm_exit
	MOV		 r4, r0				; load the pSrc pointer	
	
	CMP		 r1, #0				; check if pointer pTchY is NULL
	BEQ		 PartialNonHomTransform_asm_exit
	MOV		 r5, r1				; load the pDst pointer

	CMP		 r2, #0				; check if size is NULL
	BEQ		 PartialNonHomTransform_asm_exit
 	MOV		 r6, r2				; load array size

	MOV		 r11, r5

	vldr	 s10,[r11]			; xform[0][0]
	vldr	 s11,[r11, #4]		; xform[0][1]
	vldr	 s12,[r11, #8]		; xform[0][2]
	
	vldr	 s13,[r11, #16]		; xform[1][0]
	vldr	 s14,[r11, #20]		; xform[1][1]
	vldr	 s15,[r11, #24]		; xform[1][2]

	vldr	 s16,[r11, #32]		; xform[2][0]
	vldr	 s17,[r11, #36]		; xform[2][1]
	vldr	 s18,[r11, #40]		; xform[2][2]

PartialNonHomTransform_asm_0
	LDR		 r7, [r4], #4

	vldr	 s0,[r7]			; load x into s0
	vldr	 s1,[r7, #4]		; load y into s1
	vldr	 s2,[r7, #8]		; load z into s2

;     s20 = s0        * s10         + s1        * s13         + s2        * s16
;	tmp_x = pos[i]->x * xform[0][0] + pos[i]->y * xform[1][0] + pos[i]->z * xform[2][0];
	vmul.f32 s20,s0,s10			; s20=s0*s10
	vmul.f32 s21,s1,s13			; s21=s1*s13
	vmul.f32 s22,s2,s16			; s22=s2*s16
	vadd.f32 s20,s20,s21		; s20=s20+s21
	vadd.f32 s20,s20,s22		; s20=s20+s22	
	vstr	 s20,[r7], #4		; store the distance value to the pDstn
	
;     s20 = s0        * s11         + s1        * s14         + s2        * s17
;	tmp_y = pos[i]->x * xform[0][1] + pos[i]->y * xform[1][1] + pos[i]->z * xform[2][1];
	vmul.f32 s20,s0,s11			; s20=s0*s11
	vmul.f32 s21,s1,s14			; s21=s1*s14
	vmul.f32 s22,s2,s17			; s22=s2*s17
	vadd.f32 s20,s20,s21		; s20=s20+s21
	vadd.f32 s20,s20,s22		; s20=s20+s22	
	vstr	 s20,[r7], #4		; store the distance value to the pDstn

;     s20 = s0        * s12         + s1        * s15         + s2        * s18
;	tmp_z = pos[i]->x * xform[0][2] + pos[i]->y * xform[1][2] + pos[i]->z * xform[2][2];
	vmul.f32 s20,s0,s12			; s20=s0*s12
	vmul.f32 s21,s1,s15			; s21=s1*s15
	vmul.f32 s22,s2,s18			; s22=s2*s18
	vadd.f32 s20,s20,s21		; s20=s20+s21
	vadd.f32 s20,s20,s22		; s20=s20+s22	
	vstr	 s20,[r7], #4		; store the distance value to the pDstn
		
 	SUBS	 r6, r6, #1			; decrement the counter
 	BNE		 PartialNonHomTransform_asm_0
	B PartialNonHomTransform_asm_exit

PartialNonHomTransform_asm_exit
	POP      {r4-r11}
	BX       lr
	ENDP
	
	END

