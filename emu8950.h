#ifndef _EMU8950_H_
#define _EMU8950_H_

#include <stdint.h>
#include "emuadpcm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PI 3.14159265358979323846

/* voice data */
typedef struct __OPL_PATCH {
  uint32_t TL,FB,EG,ML,AR,DR,SL,RR,KR,KL,AM,PM,WF ;
} OPL_PATCH ;

/* slot */
typedef struct __OPL_SLOT {

  int32_t type ;          /* 0 : modulator 1 : carrier */

  /* OUTPUT */
  int32_t feedback ;
  int32_t output[5] ;      /* Output value of slot */

  /* for Phase Generator (PG) */
  uint32_t *sintbl ;    /* Wavetable */
  uint32_t phase ;      /* Phase */
  uint32_t dphase ;     /* Phase increment amount */
  uint32_t pgout ;      /* output */

  /* for Envelope Generator (EG) */
  int32_t fnum ;          /* F-Number */
  int32_t block ;         /* Block */
  uint32_t tll ;	      /* Total Level + Key scale level*/
  uint32_t rks ;        /* Key scale offset (Rks) */
  int32_t eg_mode ;       /* Current state */
  uint32_t eg_phase ;   /* Phase */
  uint32_t eg_dphase ;  /* Phase increment amount */
  uint32_t egout ;      /* output */

  /* LFO (refer to OPL->*) */
  int32_t *plfo_am ;
  int32_t *plfo_pm ;

  OPL_PATCH *patch;  

} OPL_SLOT ;

/* Channel */
typedef struct __OPL_CH {

  int32_t key_status ;
  int32_t alg ;
  OPL_SLOT *mod, *car ;

} OPL_CH ;

/* OPL */
typedef struct __OPL {

  uint32_t realstep ;
  uint32_t opltime ;
  uint32_t oplstep ;

  ADPCM *adpcm;

  uint32_t adr ;

  int32_t out ;

  /* Register */
  uint8_t reg[0xff] ; 
  int32_t slot_on_flag[18] ;

  /* Rythm Mode : 0 = OFF, 1 = ON */
  int32_t rhythm_mode ;

  /* Pitch Modulator */
  int32_t pm_mode ;
  uint32_t pm_phase ;

  /* Amp Modulator */
  int32_t am_mode ;
  uint32_t am_phase ;

  /* Noise Generator */
  uint32_t noise_seed ;

  /* Channel & Slot */
  OPL_CH *ch[9] ;
  OPL_SLOT *slot[18] ;

  uint32_t mask ;

  /* Channel output 0-8:FM 9-13:RHYTHM(not implemented) 14:ADPCM */
  int16_t ch_out[15];

} OPL ;

/* Mask */
#define OPL_MASK_CH(x) (1<<(x))
#define OPL_MASK_HH (1<<9)
#define OPL_MASK_CYM (1<<10)
#define OPL_MASK_TOM (1<<11)
#define OPL_MASK_SD (1<<12)
#define OPL_MASK_BD (1<<13)
#define OPL_MASK_RHYTHM ( OPLL_MASK_HH | OPLL_MASK_CYM | OPLL_MASK_TOM | OPLL_MASK_SD | OPLL_MASK_BD )
#define OPL_MASK_PCM (1<<14)

OPL *OPL_new(uint32_t clk, uint32_t rate) ;
void OPL_set_rate(OPL *opl, uint32_t rate) ;
void OPL_reset(OPL *opl) ;
void OPL_delete(OPL *opl) ;
void OPL_writeReg(OPL *opl, uint32_t reg, uint32_t val) ;
int16_t OPL_calc(OPL *opl) ;
void OPL_writeIO(OPL *opl, uint32_t adr, uint32_t val) ;
uint32_t OPL_readIO(OPL *opl) ;
uint32_t OPL_status(OPL *opl) ;
uint32_t OPL_setMask(OPL *opl, uint32_t mask);
uint32_t OPL_toggleMask(OPL *opl, uint32_t mask);

#ifdef __cplusplus
}
#endif



#endif











