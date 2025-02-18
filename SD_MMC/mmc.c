
#include "18f452.h"
#fuses H4,NOWDT,NOPROTECT,NOWRT,NOWRTD,NOLVP,NOOSCSEN,BORV27,PUT,STVREN,NODEBUG,NOWRTB
#use delay(clock=40000000,RESTART_WDT)
#use rs232(baud=57600,parity=N,xmit=PIN_C6,rcv=PIN_C7,ERRORS)

#include "HDD Driver.c"

void main(){

int r1,i,j,error,error0,error1;
int16 rec_no;
int16 index,rec_size;
int32 offset;
char fname[32],buff0[MMC_BUFF_SIZE+1],buff1[MMC_BUFF_SIZE+1];
char c;



setup_adc_ports(NO_ANALOGS);


set_tris_c(0b10010011); //c7=rx I, c6=tx O, c5 SDO O,c4 SDI I

output_high(_CS);


printf("\r\n**** SD / MMC FAT16  Read Demo for Sonsivri **** ");
printf("\r\n");
Delay_ms(1000);
printf("\r\Now Open COUNTRY.TXT File on SD Card  ");
printf("\r\n");
Delay_ms(3000);


SETUP_SPI (SPI_MASTER |  SPI_SS_DISABLED |SPI_H_TO_L| SPI_CLK_DIV_16 | SPI_XMIT_L_TO_H);


buff0[MMC_BUFF_SIZE]=0;
buff1[MMC_BUFF_SIZE]=0;
rec_no=0;

///////// init MMC ////////////////////////////////////////
error=init_MMC(10);
if (error>0) {
goto mmc_exit;
}


printf("\n\r MMC initialized \n\r");
rec_size=MMC_BUFF_SIZE;


//strcpy(fname,"HOME\\HOME.TXT");
strcpy(fname,"COUNTRY.TXT");
rec_size=MMC_BUFF_SIZE;
error0=open_file(0,fname,rec_size);

if (error0>0) {
printf("\n\r fopen as 0 failed error=%U\n\r",error);
goto mmc_exit;
}
else printf("\n\r opened as 0 file %s with rec size %lu \n\r",fname,rec_size);





do {

error0=file_read(0,buff0);

if (error0>0 && error0<255 ) {
printf("\n\r fread 0 failed error=%U\n\r",error0);
break;
}

printf("%s",buff0);


rec_no++;

} while (error0==0);



mmc_exit:
printf("\n\r done winhex adj= %lu \n\r",winhex_adj);



while(true);


}
