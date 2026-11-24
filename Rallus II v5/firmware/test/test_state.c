// Host test: the real state.c + control.c flown through scripted flights.
// Checks that the separation charge fires just after apogee on real-motor
// profiles (dead baro, baro spike, tilted flight included) and NEVER fires
// from a handling bump or drop on an armed rocket. Exit code 1 on failure.
#include "state.h"
#include "control.h"
#include "config.h"
#include "shared.h"
#include "pyro.h"
#include "servos.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
shared_sys_t g_sys; static baro_shared_t B; static int fired; static uint32_t fired_t; static bool armpin;
void read_baro(baro_shared_t*o){*o=B;} void pyro_fire(uint32_t t){fired++;fired_t=t;}
bool gpio_get(unsigned p){return armpin;} void servos_write_roll(float d){}
// scenario: accel profile (g, proper), true altitude integrated with gravity
static int fails;
static const char* run(const char*name, float boost_g, float boost_s, float coast_drag_g, int baro_ok, float spike_at, float tilt_deg){
  memset(&g_sys,0,sizeof g_sys); state_init(); ctrl_reset(); fired=0; armpin=true; g_sys.vbat_mv=7800;
  float v=0,h=0; uint32_t t=0; uint32_t seq=0; float ct=cosf(tilt_deg*3.14159f/180);
  flight_state_t st=ST_PADIDLE; uint32_t t_lift=0; float apo_t=-1; float vfire=0;
  for(;t<120000;t++){
    imu_sample_t s={0}; s.valid=true; float tf=(t-t_lift)/1000.f;
    if(t<5000){s.az=1;} else { if(!t_lift)t_lift=t; tf=(t-t_lift)/1000.f;
      float a = tf<boost_s? boost_g : (v>0? -coast_drag_g*(v*v)/(250.f*250.f) : 0); // proper accel g
      if(h<=0 && tf>boost_s && v<=0){ s.az=1; v=0; h=0;} else { s.az=a; v+=(a-ct)*9.81f*0.001f; h+=v*ct*0.001f; if(h<0){h=0;v=0;} }
      if(apo_t<0 && tf>boost_s && v<=0) apo_t=tf; }
    if(t%20==0){ B.alt_m=120+h + ((spike_at>0 && fabsf((t-t_lift)/1000.f-spike_at)<0.05f)?40:0); B.t_ms= baro_ok? t: 0; B.seq=++seq; }
    flight_state_t n=state_step(&s,t); ctrl_step(n,&s,t);
    if(fired==1 && vfire==0){vfire=v+0.001f;}
    st=n; if(st==ST_LANDED)break; if(fired==0 && (g_sys.events&EV_PYRO_INHIBIT) && n==ST_ARMED && t>t_lift+3000){break;}
  }
  printf("%-34s fired=%d at T+%.2fs (true apogee T+%.2fs, v=%.1f m/s) inhibit=%d final=%d\n",name,fired,fired?(fired_t-t_lift)/1000.f:0,apo_t,vfire,!!(g_sys.events&EV_PYRO_INHIBIT),st);
  int flight = boost_s > 1.0f;
  int ok = flight ? (fired==1 && (fired_t-t_lift)/1000.f >= apo_t-0.6f && (fired_t-t_lift)/1000.f <= apo_t+1.5f)
                  : (fired==0 && (g_sys.events&EV_PYRO_INHIBIT) && st==ST_ARMED);
  if(!ok){fails++; printf("   ^^^ FAIL\n");}
  return 0; }
int main(void){
  run("J323 60/40 nominal 5.4g x3.0s",5.4,3.0,1.5,1,0,0);
  run("J184 60/40 slow 3.0g x4.9s",3.0,4.9,1.0,1,0,0);
  run("J580 60/40 fast 9.8g x1.75s",9.8,1.75,2.2,1,0,0);
  run("J323, baro dead",5.4,3.0,1.5,0,0,0);
  run("J323, baro spike T+8",5.4,3.0,1.5,1,8.0,0);
  run("J323, 20 deg tilt",5.4,3.0,1.5,1,0,20);
  run("handling bump 4g x0.15s",4,0.15,0,1,0,0);
  run("drop/shake 6g x0.3s",6,0.3,0,1,0,0);
  printf(fails ? "\n%d FAILED\n" : "\nall state tests passed\n", fails);
  return fails != 0; }
