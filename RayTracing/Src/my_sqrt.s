
	AREA     ||.text||, CODE, READONLY, ALIGN=4
	

;/*
;* @brief  
;* @param[in]  f    [s0]    value.
;* @return float.
;*
;* float my_sqrt_asm( float f);
;*
;*/
	EXPORT my_sqrt_asm

my_sqrt_asm PROC
;	PUSH     {r4-r11}
		
	vsqrt.f32 s0,s0				; s0=sqrt(s0)
	
my_sqrt_asm_exit
;	POP      {r4-r11}
	BX       lr
	ENDP
	
	END

