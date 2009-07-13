/******************************************************************

      uCOSStream.c
      Z-World, 2001

		This program uses a task aware isr for the rx side of the
		serial port C isr.  Bytes are sent from one task to another
		via serial port C, and status information is sent to the stdio
		window every second.

		- task0 is responsible for sending the number of bytes transmitted
		by task2 and received by task1 for the previous second.

		- task1 waits at a semaphore which is signaled by the task aware
		isr for serial port C.  The semaphore is signaled when a byte
		arrives at the serial port, and the task receives the byte and
		increments a counter of bytes received.

		- task2 sends bytes out serial port C and increments a counter of
		bytes sent.

		- spx_isr is rewritten at the end of this file.  The rx side of the
		isr is task aware such that if the semaphore task1 is waiting on
		becomes signaled due to the arrival of a byte, and a higher priority
		task is not ready to run, then a context switch to task1 will occur.
		The compiler should generate a warning since spc_isr replaces the
		isr found in rs232.lib.  This warning can be ignored.

		Make the following connections:
		----------------------------
 		1. Connect serial TXC to RXC.

See Newv251.pdf and Relv251.pdf in the samples/ucos-II directory for
information from Jean J. Labrosse pertaining to version 2.51 of uCOS-II.

******************************************************************/
#class auto 			// Change default storage class for local variables: on the stack

// The input and output buffers sizes are defined here. If these
// are not defined to be (2^n)-1, where n = 1...15, or they are
// not defined at all, they will default to 31 and a compiler
// warning will be displayed.

#define BPS 19200
#define OS_MAX_TASKS		2
#define OS_MAX_EVENTS	2
#define OS_SEM_EN 	   2

// must explicitly use ucos library
#use ucos2.lib
#use Modem_SIMCOM.lib
#use GPS_ET332.lib

// Function prototypes for tasks
void task_GPS(void* pdata);
void task_SMS(void* pdata);

// Semaphore signaled by task aware isr
OS_EVENT* serCsem;
OS_EVENT* Semaf;

void main()
{
  	// Initialize internal OS data structures
	OSInit();

	// Create the three tasks with no initial data and 512 byte stacks
	OSTaskCreate(task_GPS, NULL, 512, 1);
	OSTaskCreate(task_SMS, NULL, 512, 0);

// Inicializo el Modem
	Inicio_Modem(BPS);

   if(config_modo_txt() == ERR_CONFIG)
   	printf("\n\nNo se configuro el modem en modo texto\n\n");

   if(Nivel_Bateria() == ERR_TIEMPO)
   	printf("\n\nNo se obtuvo nivel de la Bateria\n\n");
    else printf("\n\nNivel de la Bateria: %s\n\n", respuesta);


	// clear rx and tx data buffers
	serCrdFlush();
	serCwrFlush();

	// create semaphore used by taskSMS
	serCsem = OSSemCreate(0);
   ///Creo semaforos para el arreglo posiciones.
   Semaf = OSSemCreate(1);

   // display start message
	printf("*********************************\n");
	printf("Start\n");
	printf("*********************************\n");

	// begin multi-tasking (execution will be transferred to task0)
	OSStart();
}

void task_GPS(void* pdata)
{
	auto INT8U err_semaf;
   auto int maxSize;
   auto char sentence[TAZA];
   auto int n;
   auto int lei_pos;             /// Variable que me dice si interprete bien el mensaje del GPS de la posición.
   auto int lei_time;            /// Variable que me dice si interprete bien el mensaje del GPS del timepo UTC.
   auto struct tm *newtime;   ///Almacena la hora en la forma (*newtimw).tm_hour, (*newtime).tm_min y (*newtime).tm_sec.
   auto GPSPosition *newpos;


   ind = 0;
   lei_pos = 1;
   lei_time = 1;
   maxSize = sizeof(sentence);

   InicializarGPS();
   while(1)
	{
      OSTimeDly(3 * OS_TICKS_PER_SEC);
      if ((n = serDread(sentence, maxSize, TIMEOUT)) > 0)   /// Leo lo que me manda el GPS.
      {
      		n=0;   //para que no se vuelva a procesar los mismos datos del GPS
         	lei_pos = gps_get_position(newpos, sentence);
        		lei_time = gps_get_utc(newtime, sentence);
            if(!lei_pos && !lei_time)
            {
           		if(ind<TAM)
               {
                  OSSemPend(Semaf, 0, &err_semaf);

               	posiciones[ind].lat_degrees = newpos->lat_degrees;
               	posiciones[ind].lon_degrees = newpos->lon_degrees;
               	posiciones[ind].lat_minutes = (int) newpos->lat_minutes;
              	   posiciones[ind].lon_minutes = (int) newpos->lon_minutes;
              	   posiciones[ind].lat_seg = (newpos->lat_minutes -  posiciones[ind].lat_minutes)*60;
               	posiciones[ind].lon_seg = (newpos->lon_minutes -   posiciones[ind].lon_minutes)*60;
               	posiciones[ind].lat_direction = newpos->lat_direction;
               	posiciones[ind].lon_direction = newpos->lon_direction;
               	posiciones[ind].hora = newtime->tm_hour;
               	posiciones[ind].min = newtime->tm_min;
               	posiciones[ind].seg = newtime->tm_sec;
               	posiciones[ind].dia = newtime->tm_mday;
               	posiciones[ind].mes = newtime->tm_mon;
               	posiciones[ind].año = newtime->tm_year;
                  cambio_de_hora(posiciones[ind].dia, posiciones[ind].mes, posiciones[ind].año, posiciones[ind].hora);
                  ind++;

           			OSSemPost(Semaf);
         		}else{
               		OSSemPend(Semaf, 0, &err_semaf);
               		ind=0;
               		OSSemPost(Semaf);
            		  }

         	}

      }
   } // fin del loop
}  /// Fin de la tarea GPS.

void task_SMS(void* pdata)
{
	auto INT8U err;
   auto int car_ser;
   auto char cns[200];
   auto char msj[200];
   auto int i;
   auto char *num;
   static char num_msg[4];
   GPS_Datos g1;  ///  Arreglo de la estructura con todo los datos del GPS.

	while(1)
	{
		// wait for a byte to arrive at serial port.  This semaphore
		// will be signaled by the task aware isr for serial port C
		// at the end of this file.
		OSSemPend(serCsem, 0, &err);

       i = 0;

       while(serCrdFree() != CINBUFSIZE)
       {
		  car_ser = serCgetc();
        cns[i] = car_ser;
        printf("%c", cns[i] );
        i++;
       }

       if( (num = strstr(cns, "+CMTI:")) != '\0'  )
       {
	        //printf("%c ",*num);
	        num += 12;
	        //printf(" %c\n",*num);
	        num_msg[0]=num[0];
	        num_msg[1]=num[1];
	        num_msg[2]='\r';
	        num_msg[3]='\0';
           cns[0]='\0'; // que no se vuelva a procesar el mismo sms.

           Recibir_SMS(num_msg);

           if(Procesar_SMS(txt_msj) == PARAM_OK)
           {
               if(ind>0)
               {
               	g1 = GPSgetDatos(ind); /// Obtengo la posición actual en forma de estructura.

                  /*printf("\n\n\n\nNUMERO-%d\n\nLATITUD: %d %d'",ind,g1.lat_degrees,g1.lat_minutes);
           			printf("%f'' %c \n" , g1.lat_seg, g1.lat_direction);
           	 		printf("LONGITUD: %d %d'",g1.lon_degrees,g1.lon_minutes);
           	 		printf("%f'' %c \n",g1.lon_seg, g1.lon_direction);
	          		printf("TIEMPO:    %d/0%d/0%d  ", g1.dia, g1.mes, g1.año);
             		printf("%d:%d:%d   \n\n", g1.hora, g1.min, g1.seg);  */
                  sprintf(msj,"LATITUD:\n%d %d' %f'' %c\nLONGITUD:\n%d %d' %f'' %c\nTIEMPO:\n%d/0%d/0%d\n%d:%d:%d\032",
                  				g1.lat_degrees, g1.lat_minutes, g1.lat_seg, g1.lat_direction,
                              g1.lon_degrees, g1.lon_minutes, g1.lon_seg, g1.lon_direction,
                              g1.dia, g1.mes, g1.año, g1.hora, g1.min, g1.seg);
                  if(Enviar_SMS(num_cel, msj) == RESP_OK)   //enviar respuesta con coordenadas
         		  		printf("Mensaje respuesta de coordenadas enviado\n");
         		   else printf("Mensaje respuesta de coordenadas no enviado\n");

               }

      	  }
           else if(Enviar_SMS(num_cel, MSJ_ERR_PARAM) == RESP_OK) //enviar mensaje indicando error de parametro
                     printf("Mensaje respuesta de error enviado\n");
                  else printf("Mensaje respuesta de error no enviado\n");

           Borrar_SMS(num_msg);    //borra el mensaje previamente procesado

       }
	} //Fin del loop
} //Fin de la tarea SMS



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

