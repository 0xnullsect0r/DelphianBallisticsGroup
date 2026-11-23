// Host test: the real flashlog.c against a simulated W25Q128 that behaves
// like the chip (page programs wrap inside the 256-byte page, programming
// only clears bits, erase is per 4 KB sector). Writes 38-byte records over
// a dirty chip, cuts power without finalizing, reboots, appends, erases.
// W25Q simulator: page-program wraps inside the page, programming only clears bits.
#include "flashlog.h"
#include "config.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include "hardware/spi.h"
#include "pico/stdlib.h"
static uint8_t *mem; static bool csl=true; static uint8_t cmd; static int nb; static uint32_t addr; static bool wel; static uint32_t now; static int erases;
spi_inst_t *spi1;
absolute_time_t get_absolute_time(void){return now;} uint32_t to_ms_since_boot(absolute_time_t t){return (uint32_t)t;}
void sleep_ms(uint32_t m){now+=m;} void gpio_init(uint p){} void gpio_set_dir(uint p,bool o){} bool gpio_get(uint p){return 0;}
void gpio_put(uint pin,bool v){ if(pin!=PIN_CS_FLASH)return; if(!v){nb=0;} csl=v; }
static uint8_t xfer(uint8_t out){
  uint8_t in=0xFF;
  if(nb==0){cmd=out; if(cmd==0x06)wel=true;}
  else switch(cmd){
   case 0x9F: in = nb==1?0xEF: nb==2?0x40:0x18; break;
   case 0x05: in=0; break;
   case 0x03: if(nb<=3){addr=(addr<<8|out)&0xFFFFFF;} else {in=mem[addr]; addr=(addr+1)&0xFFFFFF;} break;
   case 0x02: if(nb<=3){addr=(addr<<8|out)&0xFFFFFF;} else {assert(wel); mem[addr]&=out; addr=(addr&~0xFFu)|((addr+1)&0xFF);} break;
   case 0x20: if(nb<=3){addr=(addr<<8|out)&0xFFFFFF; if(nb==3){assert(wel); memset(mem+(addr&~0xFFFu),0xFF,4096); erases++;}} break;
  }
  nb++; return in;
}
int spi_write_blocking(spi_inst_t*s,const uint8_t*p,size_t n){for(size_t i=0;i<n;i++)xfer(p[i]);return n;}
int spi_read_blocking(spi_inst_t*s,uint8_t f,uint8_t*d,size_t n){for(size_t i=0;i<n;i++)d[i]=xfer(f);return n;}
int spi_write_read_blocking(spi_inst_t*s,const uint8_t*t,uint8_t*r,size_t n){for(size_t i=0;i<n;i++)r[i]=xfer(t[i]);return n;}
static log_record_t mk(uint32_t i){log_record_t r; memset(&r,0,sizeof r); r.t_ms=i; r.d1=i*2654435761u; r.ax_mg=(int16_t)i; r.state=i%7; r.events=i&3; return r;}
static void check(uint32_t n){ assert(flashlog_count()==n); for(uint32_t i=0;i<n;i++){log_record_t r,e=mk(i); assert(flashlog_read(i,&r)); if(memcmp(&r,&e,sizeof r)){printf("MISMATCH at %u\n",i); exit(1);} } }
int main(void){
  mem=malloc(16<<20); 
  for(size_t i=0;i<(16u<<20);i++) mem[i]=rand();           // dirty chip: forces erase path
  assert(flashlog_init());
  uint32_t n=0;
  for(;n<20000;n++){log_record_t r=mk(n); assert(flashlog_append(&r)); if(n%97==0)flashlog_flush();}
  flashlog_flush(); check(n); printf("phase1 ok: %u recs, %d erases\n",n,erases);
  // power cut without finalize: header lags; reboot must recover every flushed record
  assert(flashlog_init()); printf("after reboot count=%u (wrote %u)\n",flashlog_count(),n); check(n);
  for(;n<26000;n++){log_record_t r=mk(n); assert(flashlog_append(&r));}
  flashlog_finalize(); check(n);
  assert(flashlog_erase_used()); assert(flashlog_count()==0);
  int e0=erases; n=0;
  for(;n<30000;n++){log_record_t r=mk(n); assert(flashlog_append(&r));}
  flashlog_finalize(); check(n);
  printf("phase3 ok: in-flight data-sector erases after ground erase = %d (header ckpts=%u)\n", erases-e0, (30000u*38)/(256*1024)+1);
  return 0; }
