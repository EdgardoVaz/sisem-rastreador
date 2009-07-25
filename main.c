#define OS_MAX_TASKS		2
#define OS_MAX_EVENTS	1
#define OS_SEM_EN 	   1

#use Modem_SIMCOM.lib
#use GPS_ET332.lib
#use ucos2.lib

#define BPS  19200
#define tout    10
#define ON       1
#define OFF      0

//Prototipos para las tasks
	void task_GPS(void* pdata);
	void task_SMS(void* pdata);

//Semaforo señalado por task aware isr
	OS_EVENT* serCsem;


shared char msj[TAM];
shared int coord_ok;

void main(){

// Inicializa la estructura interna del OS.
	OSInit();

// Crea dos tareas sin valores iniciales y 512 byte de stacks
	OSTaskCreate(task_GPS, NULL, 512, 1);
	OSTaskCreate(task_SMS, NULL, 512, 0);

// Crea el semaforo usado por taskSMS
	serCsem = OSSemCreate(0);

// Despliega mensaje inicial del RTOS
	printf("*********************************\n");
	printf("FILIPHIDES v1.0\n");
	printf("*********************************\n");

// Arranca el sistema operativo
	OSStart();
}



void task_SMS(void* pdata)
{
	auto INT8U err;
   static char cns[100];
	auto char *num, *n;
	static char num_msg[4];
	static char txt_msj[TAM];
	static char num_cel[16];
// Inicializo el Modem
   Inicio_Modem(BPS);
// limpia el buffer rx y tx del puerto serial C
	serCrdFlush();
	serCwrFlush();

   ModoSleep(ON); //pone el modem en bajo consumo

	for(;;)
	{
		 // wait for a byte to arrive at serial port.  This semaphore
		 // will be signaled by the task aware isr for serial port C
		 // at the end of this file.
		 OSSemPend(serCsem, 0, &err);
       OSTimeDly(10);  //espera 10 ticks de reloj del RTOS (1Tick = 10ms).
       n_gsm = serCread(cns, sizeof(cns), tout);
       cns[n_gsm]='\0';     //para no volver a procesar otra vez lo que quedo de la ocación anterior
       printf("%s", cns);
       n_gsm = 0;
       num = cns;
       while((num = strstr(num, "+CMTI:")) != '\0')    //proceso todos los sms leidos en una instancia
       {
       		num += 12;
            num_msg[0]=num[0];     //Selecciona el nº de sms recibido
	        	num_msg[1]=num[1];
	        	num_msg[2]='\r';
	        	num_msg[3]='\0';

            ModoSleep(OFF);   //despierto el modem
            OSTimeDly(3);     //espera 3 ticks de reloj del RTOS (1Tick = 10ms) para despertarse.

            Recibir_SMS(num_msg, txt_msj);
            switch(Procesar_SMS(num_cel, txt_msj))
            {
            	case PARAM_OK:
                             		if(coord_ok)
               				  		{
                        				Enviar_SMS(num_cel, msj);   //enviar respuesta con coordenadas
                        				OSTimeDlySec(10);
                        				if(Respuesta_Modem(ESPERO_OK, respuesta, TIEMPO) == RESP_OK)
                        					printf("Mensaje con coordenadas, enviado\n");
           		   						else printf("Mensaje con coordenadas, no enviado\n");
                   					}
                                 else
                                 {
                                    Enviar_SMS(num_cel, MSJ_NO_COORD);   //enviar respuesta con coordenadas
                        				OSTimeDlySec(10);
                        				if(Respuesta_Modem(ESPERO_OK, respuesta, TIEMPO) == RESP_OK)
                        					printf("No hay coordenadas, enviado\n");
           		   						else printf("No hay coordenadas, no enviado\n");
                                 }
                              	break;
               case ERR_PARAM:
                              	Enviar_SMS(num_cel, MSJ_ERR_PARAM); //enviar mensaje indicando error de parametro
               	 					OSTimeDlySec(10);
                   					if(Respuesta_Modem(ESPERO_OK, respuesta, TIEMPO) == RESP_OK)
               	    					printf("Mensaje parametro incorrecto, enviado\n");
               	 					else printf("Mensaje parametro incorrecto, no enviado\n");
                              	break;
               case ERR_NUM_CEL:
                                 printf("Error al obtener el numero de celular\n\n");
                                 break;
               default:
                                 printf("Error grave al procesar el mensaje\n\n");
            }
            Borrar_SMS(num_msg); //Borra el mensaje previamente procesado
       }
       ModoSleep(ON); //pone el modem en bajo consumo nuevamente.
   }//Fin del loop
}//Fin task_SMS



void task_GPS(void* pdata)
{
	static char data[TAM];

// Inicializo el GPS
   InicializarGPS();
// limpia el buffer rx y tx del puerto serial D
	serDrdFlush();
	serDwrFlush();
   for(;;)
	{
   	 OSTimeDly(3 * OS_TICKS_PER_SEC);
   	 n_gps = serDread(data, sizeof(data), tout);
   	 data[n_gps]='\0';
   	 coord_ok = ProcesarGPS(data, n_gps);
   	 sprintf(msj, "Latitud: %s\nLongitud: %s\nHora: %s\nFecha: %s\n\032", latitud, longitud, hora_utc, fecha);
   	 n_gps = 0;

   }//Fin del loop
}//Fin task_GPS



// --------- serial port c task aware interrupt ---------------------------- //

#asm nodebug root
;
; spc_isr
;
spx_isr::
   push	af					; 7, 	restore registers needed by isr
   push	bc					; 7
   push	de					; 7
   push	hl					; 7
   ld		hl, lxpc
   push	hl
   push	iy

   ; IX = serXdata
   ld		hl, (ix+_sxd+sxdr)

   ld		iy, hl

   ioi	ld a, (iy+SxSR_OFS)
   ld		c, a
   rla							; 2
   jp		c, spx_rx			; 5

spx_tx:
   bit	2, c           ; Tx busy? bit 2 shifted via rla
   jr		nz, spx_txbusy
spx_txidle:             ; Tx idle, safe for enable/disable transition
   _READ_FLAG_(SER_FLAG_LONGSTOP)
   jr		nz, spx_dolongstop	;handle request for a loooong (2 byte) stop bit

spx_trytosend:
   _CALL_(xEnable)
   call	spx_txload				;CTS is on, so try to load a normal byte
   jr		spx_donomore

spx_dolongstop:
   _CLEAR_FLAG_(SER_FLAG_LONGSTOP)     ;clear out the long stop flag
   jr		spx_donomore

spx_txbusy:
   ; check to make sure tx is enabled
   ; if not, dummy character still needs to clear out
#ifdef SER_NO_SHADOWS
   ld		hl, (ix+_sxd+pxfr)
   ioi	ld hl, (hl)
#else
   ld		hl, (ix+_sxd+pxfrshadow)
   ld		hl, (hl)
#endif
   ld		a, (ix+_sxd+xdrive_txd)
   and	l
   jr		z, spx_donomore
spx_txallgood:
    call	 spx_txload					; will load a good byte, it there is one to
    										; be loaded
spx_donomore:
   ioi	ld (iy+SxSR_OFS), a     ;	clear interrupt source
	jp		spc_finish

spx_rx:
	ld		hl, bios_intnesting
	inc	(hl)

   ioi	ld a, (iy+SxDR_OFS)		; 11,	receive the character
   bool	hl					; 2,	place character on circular buffer
   ld		L, a				; 2,		throw it away if no room
   ld		b, a				; save for later possible parity check
   _READ_FLAG_(SER_FLAG_SEVENBIT)
   jr		z, spx_rxcontinue
   ld		a, L
   and	0x7f
   ld		L, a				;mask out MSB for 7 bit data
spx_rxcontinue:
   push	bc				; save byte copy and status register copy
   push	hl            ; 10
   ld		hl, (ix+_sxd+inbuf)
   push	hl            ; 10
   lcall	cbuf_putch	; 12+...
   add	sp, 4          ; 4

; signal ucos semaphore that byte has arrived
	 ld	 a, (OSRunning)
	 ld	 b, a
	 or	 a
	 jr    z, nopost

   push hl
   ld hl, (ix+_sxd+sxdr)
   ld de, 0000000011100000b     ;0x00E0
   cp hl, de                    ; interrumpio serie C
   pop hl
   jp nz, nopost

    push  ix
	 push  iy						; OSSemPost trashes iy (other regs already saved)
    exx
    push  hl
    exx
	 ld	 hl, (serCsem)
    push	 hl
    call	 OSSemPost
	 add	 sp, 2
    exx
    pop   hl
    exx
    pop   iy
    pop   ix
nopost:

   pop	bc
   bit	5, c				;test for receiver overrun
   jr		z, spx_checkparity
   _SET_FLAG_(SER_FLAG_OVERRUN)

spx_checkparity:
   _READ_FLAG_(SER_FLAG_PARITY)     ; see if we need to check parity of
    										; incoming byte
   jr		z, spx_rx_flowcontrol
   _READ_FLAG_(SER_FLAG_SEVENBIT)
   jr		nz, spx_check7bit
spx_check8bit:
#if (CPU_ID_MASK(_CPU_ID_) >= R4000)
   bit	4, c
   jr		z, spx_rx_flowcontrol
   _SET_FLAG_(SER_FLAG_PARITYERROR)		; signal a parity error
   jr		spx_rx_flowcontrol
#else
   ld		l, b
   ld		a, (ix+_sxd+paritytype)
   ld		h, a
   lcall	ser8_getparity
   bool	hl
   jr		z, spx_check9low
spx_check9high:						; 9th parity bit should be high,
   										; otherwise we have a parity error
   bit	6, c							; C reg should still have a snapshot of
    										; the status register, checking bit 9
   jr		z, spx_rx_flowcontrol	; no 9th bit detected (9th bit high),
    										; so we are OK
   _SET_FLAG_(SER_FLAG_PARITYERROR)		; signal a parity error
   jr		spx_rx_flowcontrol
spx_check9low:
   bit	6, c
   jr		nz, spx_rx_flowcontrol	; 9th bit detected (9th bit low),
    										; which is what we expect
   _SET_FLAG_(SER_FLAG_PARITYERROR)		; signal a parity error
   jr		spx_rx_flowcontrol
#endif //(CPU_ID_MASK(_CPU_ID_) >= R4000)
spx_check7bit:
   ld		l, b
   ld		a, (ix+_sxd+paritytype)
   ld		h, a
   lcall	ser7_checkparity
   bool	hl
   jr		nz, spx_rx_flowcontrol
   _SET_FLAG_(SER_FLAG_PARITYERROR)		; signal a parity error
spx_rx_flowcontrol:
spx_rx0:

	ld		a, (OSRunning)
	or		a
	jr		z, spc_decnesting

	call	OSIntExit

spc_decnesting:
	push	ip
	ipset	1
	ld		hl, bios_intnesting
	dec	(hl)
	jr		nz, spc_finish_rx

	ld		a, (bios_swpend)
	or		a
	jr		z, spc_finish_rx

	ex		af,af'
	push	af
	exx
	push	hl
	push	de
	push	bc
	push	iy
	push	ix

	lcall	bios_intexit

	pop	ix
	pop	iy
	pop	bc
	pop	de
	pop	hl
	exx
	pop	af
	ex		af,af'

spc_finish_rx:
	pop	ip

spc_finish:

   ipres               ; 4
   pop	iy
   pop	hl            ; 7,  restore registers needed by isr
   ld		lxpc, hl
   pop	hl
   pop	de            ; 7
   pop	bc            ; 7
   pop	af            ; 7
   ret                 ; 13
#endasm


