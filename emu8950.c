/****************************************************************************

  emu8950.c -- Y8950 emulator : written by Mitsutaka Okazaki 2003-2016

  Note: rythm channels are not yet implemented. 

  2001 05-19 : Version 0.10 -- Test release.
  2001 ??-?? : Version 0.11 -- Added ADPCM.
  2002 03-02 : Version 0.12 -- Removed OPL_init & OPL_close.
  2003 09-19 : Version 0.13 -- Added OPL_setMask & OPL_toggleMask.
  2016 09-06 : Version 0.14 -- Support per-channel output.

  References: 
    fmopl.c        -- 1999,2000 written by Tatsuyuki Satoh.
    s_opl.c        -- 2001 written by Mamiya.
    fmgen.cpp      -- 1999,2000 written by cisc.
    MSX-Datapack
    YMU757 data sheet
    YM2143 data sheet

*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "emu8950.h"

/* Size of Sintable ( 1 -- 18 can be used, but 7 -- 14 recommended.)*/
#define PG_BITS 10
#define PG_WIDTH (1<<PG_BITS)

/* Phase increment counter */
#define DP_BITS 19
#define DP_WIDTH (1<<DP_BITS)
#define DP_BASE_BITS (DP_BITS - PG_BITS)

#define ALIGN(d,SS,SD) (d*(int32_t)(SS/SD))
#define ALIGN2(d,SS,SD) (d*(int32_t)(SS/SD))

/* Dynamic range */
#define DB_STEP 0.1875
#define DB_BITS 9
#define DB_MUTE (1<<DB_BITS)

/* Dynamic range of envelope */
#define EG_STEP 0.1875
#define EG_BITS 9
#define EG_MUTE (1<<EB_BITS)

/* Dynamic range of total level */
#define TL_STEP 0.75
#define TL_BITS 6
#define TL_MUTE (1<<TL_BITS)

/* Dynamic range of sustine level */
#define SL_STEP 3.0
#define SL_BITS 4
#define SL_MUTE (1<<SL_BITS)

/* Bits for liner value */
#define DB2LIN_AMP_BITS 11
#define SLOT_AMP_BITS (DB2LIN_AMP_BITS)

/* Bits for envelope phase incremental counter */
#define EG_DP_BITS 23
#define EG_DP_WIDTH (1<<EG_DP_BITS)

/* Bits for Pitch and Amp modulator */
#define PM_PG_BITS 8
#define PM_PG_WIDTH (1<<PM_PG_BITS)
#define PM_DP_BITS 16
#define PM_DP_WIDTH (1<<PM_DP_BITS)
#define AM_PG_BITS 8
#define AM_PG_WIDTH (1<<AM_PG_BITS)
#define AM_DP_BITS 16
#define AM_DP_WIDTH (1<<AM_DP_BITS)

/* PM table is calcurated by PM_AMP * pow(2,PM_DEPTH*sin(x)/1200) */
#define PM_AMP_BITS 8
#define PM_AMP (1<<PM_AMP_BITS)

/* PM speed(Hz) and depth(cent) */
#define PM_SPEED 6.4
#define PM_DEPTH (13.75/2)
#define PM_DEPTH2 13.75

/* AM speed(Hz) and depth(dB) */
#define AM_SPEED 3.7
#define AM_DEPTH 1.0
#define AM_DEPTH2 4.8

/* Cut the lower b bit(s) off. */
#define HIGHBITS(c,b) ((c)>>(b))

/* Leave the lower b bit(s). */
#define LOWBITS(c,b) ((c)&((1<<(b))-1))

/* Expand x which is s bits to d bits. */
#define EXPAND_BITS(x,s,d) ((x)<<((d)-(s)))

/* Expand x which is s bits to d bits and fill expanded bits '1' */
#define EXPAND_BITS_X(x,s,d) (((x)<<((d)-(s)))|((1<<((d)-(s)))-1))

/* Adjust envelope speed which depends on sampling rate. */
#define rate_adjust(x) (uint32_t)((double)(x)*clk/72/rate + 0.5)        /* +0.5 to round */

#define MOD(x) ch[x]->mod
#define CAR(x) ch[x]->car

#define SLOT_BD1 12
#define SLOT_BD2 13
#define SLOT_HH 14
#define SLOT_SD 15
#define SLOT_TOM 16
#define SLOT_CYM 17

/* Sampling rate */
static uint32_t rate;
/* Input clock */
static uint32_t clk;

/* WaveTable for each envelope amp */
static uint32_t fullsintable[PG_WIDTH];

/* LFO Table */
static int32_t pmtable[2][PM_PG_WIDTH];
static int32_t amtable[2][AM_PG_WIDTH];

static uint32_t pm_dphase;
static int32_t lfo_pm;
static uint32_t am_dphase;
static int32_t lfo_am;
static uint32_t whitenoise;

/* dB to Liner table */
static int32_t DB2LIN_TABLE[(DB_MUTE + DB_MUTE) * 2];

/* Liner to Log curve conversion table (for Attack rate). */
static uint32_t AR_ADJUST_TABLE[1 << EG_BITS];

/* Definition of envelope mode */
enum OPL_EG_STATE
{ SETTLE, ATTACK, DECAY, SUSHOLD, SUSTINE, RELEASE, FINISH };

/* Phase incr table for Attack */
static uint32_t dphaseARTable[16][16];
/* Phase incr table for Decay and Release */
static uint32_t dphaseDRTable[16][16];

/* KSL + TL Table */
static uint32_t tllTable[16][8][1 << TL_BITS][4];
static int32_t rksTable[2][8][2];

/* Phase incr table for PG */
static uint32_t dphaseTable[1024][8][16];

/***************************************************
 
                  Create tables
 
****************************************************/
static inline int32_t
Min (int32_t i, int32_t j)
{
  if (i < j)
    return i;
  else
    return j;
}

/* Table for AR to LogCurve. */
static void
makeAdjustTable (void)
{
  int32_t i;

  AR_ADJUST_TABLE[0] = (1 << EG_BITS);
  for (i = 1; i < 1 << EG_BITS; i++)
    AR_ADJUST_TABLE[i] = (uint32_t) ((double) (1 << EG_BITS) - 1 - (1 << EG_BITS) * log (i) / log (1 << EG_BITS)) >> 1;
}

/* Table for dB(0 -- (1<<DB_BITS)) to Liner(0 -- DB2LIN_AMP_WIDTH) */
static void
makeDB2LinTable (void)
{
  int32_t i;

  for (i = 0; i < DB_MUTE + DB_MUTE; i++)
  {
    DB2LIN_TABLE[i] = (int32_t) ((double) ((1 << DB2LIN_AMP_BITS) - 1) * pow (10, -(double) i * DB_STEP / 20));
    if (i >= DB_MUTE)
      DB2LIN_TABLE[i] = 0;
    DB2LIN_TABLE[i + DB_MUTE + DB_MUTE] = -DB2LIN_TABLE[i];
  }
}

/* Liner(+0.0 - +1.0) to dB((1<<DB_BITS) - 1 -- 0) */
static int32_t
lin2db (double d)
{
  if (d == 0)
    return (DB_MUTE - 1);
  else
    return Min (-(int32_t) (20.0 * log10 (d) / DB_STEP), DB_MUTE - 1);  /* 0 -- 128 */
}

/* Sin Table */
static void
makeSinTable (void)
{
  int32_t i;

  for (i = 0; i < PG_WIDTH / 4; i++)
    fullsintable[i] = lin2db (sin (2.0 * PI * i / PG_WIDTH));

  for (i = 0; i < PG_WIDTH / 4; i++)
    fullsintable[PG_WIDTH / 2 - 1 - i] = fullsintable[i];

  for (i = 0; i < PG_WIDTH / 2; i++)
    fullsintable[PG_WIDTH / 2 + i] = DB_MUTE + DB_MUTE + fullsintable[i];
}

/* Table for Pitch Modulator */
static void
makePmTable (void)
{
  int32_t i;

  for (i = 0; i < PM_PG_WIDTH; i++)
    pmtable[0][i] = (int32_t) ((double) PM_AMP * pow (2, (double) PM_DEPTH * sin (2.0 * PI * i / PM_PG_WIDTH) / 1200));

  for (i = 0; i < PM_PG_WIDTH; i++)
    pmtable[1][i] = (int32_t) ((double) PM_AMP * pow (2, (double) PM_DEPTH2 * sin (2.0 * PI * i / PM_PG_WIDTH) / 1200));
}

/* Table for Amp Modulator */
static void
makeAmTable (void)
{
  int32_t i;

  for (i = 0; i < AM_PG_WIDTH; i++)
    amtable[0][i] = (int32_t) ((double) AM_DEPTH / 2 / DB_STEP * (1.0 + sin (2.0 * PI * i / PM_PG_WIDTH)));

  for (i = 0; i < AM_PG_WIDTH; i++)
    amtable[1][i] = (int32_t) ((double) AM_DEPTH2 / 2 / DB_STEP * (1.0 + sin (2.0 * PI * i / PM_PG_WIDTH)));
}

/* Phase increment counter table */
static void
makeDphaseTable (void)
{
  uint32_t fnum, block, ML;
  uint32_t mltable[16] =
    { 1, 1 * 2, 2 * 2, 3 * 2, 4 * 2, 5 * 2, 6 * 2, 7 * 2, 8 * 2, 9 * 2, 10 * 2, 10 * 2, 12 * 2, 12 * 2, 15 * 2, 15 * 2 };

  for (fnum = 0; fnum < 1024; fnum++)
    for (block = 0; block < 8; block++)
      for (ML = 0; ML < 16; ML++)
        dphaseTable[fnum][block][ML] = rate_adjust ((((fnum * mltable[ML]) << block) >> (21 - DP_BITS)));
}

static void
makeTllTable (void)
{
#define dB2(x) (uint32_t)((x)*2)

  static uint32_t kltable[16] = {
    dB2 (0.000), dB2 (9.000), dB2 (12.000), dB2 (13.875), dB2 (15.000), dB2 (16.125), dB2 (16.875), dB2 (17.625),
    dB2 (18.000), dB2 (18.750), dB2 (19.125), dB2 (19.500), dB2 (19.875), dB2 (20.250), dB2 (20.625), dB2 (21.000)
  };

  int32_t tmp;
  int32_t fnum, block, TL, KL;

  for (fnum = 0; fnum < 16; fnum++)
    for (block = 0; block < 8; block++)
      for (TL = 0; TL < 64; TL++)
        for (KL = 0; KL < 4; KL++)
        {
          if (KL == 0)
          {
            tllTable[fnum][block][TL][KL] = ALIGN (TL, TL_STEP, EG_STEP);
          }
          else
          {
            tmp = kltable[fnum] - dB2 (3.000) * (7 - block);
            if (tmp <= 0)
              tllTable[fnum][block][TL][KL] = ALIGN (TL, TL_STEP, EG_STEP);
            else
              tllTable[fnum][block][TL][KL] = (uint32_t) ((tmp >> (3 - KL)) / EG_STEP) + ALIGN (TL, TL_STEP, EG_STEP);
          }

        }
}



/* Rate Table for Attack */
static void
makeDphaseARTable (void)
{
  int32_t AR, Rks, RM, RL;

  for (AR = 0; AR < 16; AR++)
    for (Rks = 0; Rks < 16; Rks++)
    {
      RM = AR + (Rks >> 2);
      if (RM > 15)
        RM = 15;
      RL = Rks & 3;
      switch (AR)
      {
      case 0:
        dphaseARTable[AR][Rks] = 0;
        break;
      case 15:
        dphaseARTable[AR][Rks] = EG_DP_WIDTH;
        break;
      default:
        dphaseARTable[AR][Rks] = rate_adjust ((3 * (RL + 4) << (RM + 1)));
        break;
      }

    }
}

/* Rate Table for Decay */
static void
makeDphaseDRTable (void)
{
  int32_t DR, Rks, RM, RL;

  for (DR = 0; DR < 16; DR++)
    for (Rks = 0; Rks < 16; Rks++)
    {
      RM = DR + (Rks >> 2);
      RL = Rks & 3;
      if (RM > 15)
        RM = 15;
      switch (DR)
      {
      case 0:
        dphaseDRTable[DR][Rks] = 0;
        break;
      default:
        dphaseDRTable[DR][Rks] = rate_adjust ((RL + 4) << (RM - 1));
        break;
      }

    }
}

static void
makeRksTable (void)
{
  int32_t fnum9, block, KR;

  for (fnum9 = 0; fnum9 < 2; fnum9++)
    for (block = 0; block < 8; block++)
      for (KR = 0; KR < 2; KR++)
      {
        if (KR != 0)
          rksTable[fnum9][block][KR] = (block << 1) + fnum9;
        else
          rksTable[fnum9][block][KR] = block >> 1;
      }
}

/************************************************************

                      Calc Parameters

************************************************************/

static inline uint32_t
calc_eg_dphase (OPL_SLOT * slot)
{
  switch (slot->eg_mode)
  {
  case ATTACK:
    return dphaseARTable[slot->patch->AR][slot->rks];

  case DECAY:
    return dphaseDRTable[slot->patch->DR][slot->rks];

  case SUSHOLD:
    return 0;

  case SUSTINE:
    return dphaseDRTable[slot->patch->RR][slot->rks];

  case RELEASE:
    if (slot->patch->EG)
      return dphaseDRTable[slot->patch->RR][slot->rks];
    else
      return dphaseDRTable[7][slot->rks];

  case FINISH:
    return 0;

  default:
    return 0;
  }
}

/*************************************************************

                    opl internal interfaces

*************************************************************/
#define SLOT_BD1 12
#define SLOT_BD2 13
#define SLOT_HH 14
#define SLOT_SD 15
#define SLOT_TOM 16
#define SLOT_CYM 17

#define UPDATE_PG(S)  (S)->dphase = dphaseTable[(S)->fnum][(S)->block][(S)->patch->ML]
#define UPDATE_TLL(S) \
  ((S)->tll = tllTable[((S)->fnum)>>6][(S)->block][(S)->patch->TL][(S)->patch->KL])

#define UPDATE_RKS(S) (S)->rks = rksTable[((S)->fnum)>>9][(S)->block][(S)->patch->KR]
#define UPDATE_EG(S)  (S)->eg_dphase = calc_eg_dphase(S)
#define UPDATE_ALL(S) \
  UPDATE_PG(S) ;\
  UPDATE_TLL(S) ;\
  UPDATE_RKS(S) ;\
  UPDATE_EG(S)                  /* EG should be last */

/* Slot key on  */
static inline void
slotOn (OPL_SLOT * slot)
{
  slot->eg_mode = ATTACK;
  slot->phase = 0;
  slot->eg_phase = 0;
}

/* Slot key off */
static inline void
slotOff (OPL_SLOT * slot)
{
  if (slot->eg_mode == ATTACK)
    slot->eg_phase = EXPAND_BITS (AR_ADJUST_TABLE[HIGHBITS (slot->eg_phase, EG_DP_BITS - EG_BITS)], EG_BITS, EG_DP_BITS);

  slot->eg_mode = RELEASE;
}

/* Channel key on */
static inline void
keyOn (OPL * opl, int32_t i)
{
  slotOn (opl->MOD (i));
  slotOn (opl->CAR (i));
  opl->ch[i]->key_status = 1;
}

/* Channel key off */
static inline void
keyOff (OPL * opl, int32_t i)
{
  slotOff (opl->MOD (i));
  slotOff (opl->CAR (i));
  opl->ch[i]->key_status = 0;
}

/**
static inline void
keyOn_BD (OPL * opl)
{
  keyOn (opl, 6);
}
static inline void
keyOn_SD (OPL * opl)
{
  if (!opl->slot_on_flag[SLOT_SD])
    slotOn (opl->CAR (7));
}
static inline void
keyOn_TOM (OPL * opl)
{
  if (!opl->slot_on_flag[SLOT_TOM])
    slotOn (opl->MOD (8));
}
static inline void
keyOn_HH (OPL * opl)
{
  if (!opl->slot_on_flag[SLOT_HH])
    slotOn (opl->MOD (7));
}
static inline void
keyOn_CYM (OPL * opl)
{
  if (!opl->slot_on_flag[SLOT_CYM])
    slotOn (opl->CAR (8));
}
static inline void
keyOff_BD (OPL * opl)
{
  keyOff (opl, 6);
}
static inline void
keyOff_SD (OPL * opl)
{
  if (opl->slot_on_flag[SLOT_SD])
    slotOff (opl->CAR (7));
}
static inline void
keyOff_TOM (OPL * opl)
{
  if (opl->slot_on_flag[SLOT_TOM])
    slotOff (opl->MOD (8));
}
static inline void
keyOff_HH (OPL * opl)
{
  if (opl->slot_on_flag[SLOT_HH])
    slotOff (opl->MOD (7));
}
static inline void
keyOff_CYM (OPL * opl)
{
  if (opl->slot_on_flag[SLOT_CYM])
    slotOff (opl->CAR (8));
}
*/

/* Set F-Number ( fnum : 10bit ) */
static inline void
setFnumber (OPL * opl, int32_t c, int32_t fnum)
{
  opl->CAR (c)->fnum = fnum;
  opl->MOD (c)->fnum = fnum;
}

/* Set Block data (block : 3bit ) */
static inline void
setBlock (OPL * opl, int32_t c, int32_t block)
{
  opl->CAR (c)->block = block;
  opl->MOD (c)->block = block;
}

/***********************************************************

                      Initializing

***********************************************************/

static void
OPL_SLOT_reset (OPL_SLOT * slot)
{
  slot->sintbl = fullsintable;
  slot->phase = 0;
  slot->dphase = 0;
  slot->output[0] = 0;
  slot->output[1] = 0;
  slot->feedback = 0;
  slot->eg_mode = FINISH;
  slot->eg_phase = EG_DP_WIDTH;
  slot->eg_dphase = 0;
  slot->rks = 0;
  slot->tll = 0;
  slot->fnum = 0;
  slot->block = 0;
  slot->pgout = 0;
  slot->egout = 0;
  memset (slot->patch, 0, sizeof (OPL_PATCH));
  UPDATE_ALL (slot);

}

static OPL_SLOT *
OPL_SLOT_new (void)
{
  OPL_SLOT *slot;
  OPL_PATCH *patch;

  patch = (OPL_PATCH *) malloc (sizeof (OPL_PATCH));
  if (patch == NULL)
    return NULL;

  slot = (OPL_SLOT *) malloc (sizeof (OPL_SLOT));
  if (slot == NULL)
  {
    free (patch);
    return NULL;
  }

  slot->patch = patch;
  return slot;
}

static void
OPL_SLOT_delete (OPL_SLOT * slot)
{
  free (slot->patch);
  free (slot);
}

static void
OPL_CH_reset (OPL_CH * ch)
{
  OPL_SLOT_reset (ch->mod);
  OPL_SLOT_reset (ch->car);
  ch->key_status = 0;
  ch->alg = 0;
}

static OPL_CH *
OPL_CH_new (void)
{
  OPL_CH *ch;
  OPL_SLOT *mod, *car;

  mod = OPL_SLOT_new ();
  if (mod == NULL)
    return NULL;

  car = OPL_SLOT_new ();
  if (car == NULL)
  {
    OPL_SLOT_delete (mod);
    return NULL;
  }

  ch = (OPL_CH *) malloc (sizeof (OPL_CH));
  if (ch == NULL)
  {
    OPL_SLOT_delete (mod);
    OPL_SLOT_delete (car);
    return NULL;
  }

  mod->type = 1;
  car->type = 1;
  ch->mod = mod;
  ch->car = car;

  OPL_CH_reset (ch);

  return ch;
}

static void
OPL_CH_delete (OPL_CH * ch)
{
  OPL_SLOT_delete (ch->mod);
  OPL_SLOT_delete (ch->car);
  free (ch);
}

static void
internal_refresh (void)
{
  makeDphaseTable ();
  makeDphaseARTable ();
  makeDphaseDRTable ();
  pm_dphase = (uint32_t) rate_adjust (PM_SPEED * PM_DP_WIDTH / (clk / 72));
  am_dphase = (uint32_t) rate_adjust (AM_SPEED * AM_DP_WIDTH / (clk / 72));
}

static void
maketables (uint32_t c, uint32_t r)
{
  if (c != clk)
  {
    clk = c;
    makePmTable ();
    makeAmTable ();
    makeDB2LinTable ();
    makeAdjustTable ();
    makeTllTable ();
    makeRksTable ();
    makeSinTable ();
  }

  if (r != rate)
  {
    rate = r;
    internal_refresh ();
  }
}

uint32_t
OPL_setMask (OPL *opl, uint32_t mask)
{
  uint32_t ret = 0;
  if(opl)
  {
    ret = opl->mask;
    opl->mask = mask;
  }  
  return ret;
}

uint32_t
OPL_toggleMask (OPL *opl, uint32_t mask)
{
  uint32_t ret = 0;
  if(opl)
  {
    ret = opl->mask;
    opl->mask ^= mask;
  }
  return ret;
}

OPL *
OPL_new (uint32_t c, uint32_t r)
{
  OPL *opl;
  OPL_CH *ch[9];
  int32_t i, j;

  maketables (c, r);

  opl = calloc (sizeof (OPL), 1);
  if (opl == NULL)
    return NULL;

  for (i = 0; i < 9; i++)
  {
    ch[i] = OPL_CH_new ();
    if (ch[i] == NULL)
    {
      for (j = i; i > 0; i++)
        OPL_CH_delete (ch[j - 1]);
      free (opl);
      return NULL;
    }
  }

  for (i = 0; i < 9; i++)
  {
    opl->ch[i] = ch[i];
    opl->slot[i * 2 + 0] = opl->ch[i]->mod;
    opl->slot[i * 2 + 1] = opl->ch[i]->car;
  }

  opl->mask = 0;

  opl->adpcm = ADPCM_new (c, r);

  OPL_reset (opl);

  return opl;
}

void
OPL_delete (OPL * opl)
{
  int32_t i;

  ADPCM_delete (opl->adpcm);

  for (i = 0; i < 9; i++)
    OPL_CH_delete (opl->ch[i]);
  free (opl);

}

/* Reset whole of opl except patch datas. */
void
OPL_reset (OPL * opl)
{
  int32_t i;

  if (!opl)
    return;

  for (i = 0; i < 9; i++)
    OPL_CH_reset (opl->ch[i]);

  opl->out = 0;

  opl->rhythm_mode = 0;
  opl->am_mode = 0;
  opl->pm_mode = 0;
  opl->pm_phase = 0;
  opl->am_phase = 0;
  opl->noise_seed = 0xffff;

  memset(opl->reg, 0, sizeof(opl->reg));
  memset(opl->slot_on_flag, 0, sizeof(opl->slot_on_flag));
  memset(opl->ch_out, 0, sizeof(opl->ch_out));

  ADPCM_reset (opl->adpcm);

}


void
OPL_set_rate (OPL * opl, uint32_t r)
{
  rate = r;
  internal_refresh ();
  ADPCM_set_rate (opl->adpcm, r);
}

/*********************************************************

                 Generate wave data

*********************************************************/
/* Convert Amp(0 to EG_HEIGHT) to Phase(0 to 4PI). */
#if ( SLOT_AMP_BITS - PG_BITS - 1 ) == 0
#define wave2_4pi(e)  (e)
#elif ( SLOT_AMP_BITS - PG_BITS - 1 ) > 0
#define wave2_4pi(e)  ( (e) >> ( SLOT_AMP_BITS - PG_BITS - 1 ))
#else
#define wave2_4pi(e)  ( (e) << ( 1 + PG_BITS - SLOT_AMP_BITS ))
#endif

/* Convert Amp(0 to EG_HEIGHT) to Phase(0 to 8PI). */
#if ( SLOT_AMP_BITS - PG_BITS - 2 ) == 0
#define wave2_8pi(e)  (e)
#elif ( SLOT_AMP_BITS - PG_BITS - 2 ) > 0
#define wave2_8pi(e)  ( (e) >> ( SLOT_AMP_BITS - PG_BITS - 2 ))
#else
#define wave2_8pi(e)  ( (e) << ( 2 + PG_BITS - SLOT_AMP_BITS ))
#endif

static inline uint32_t
mrand (uint32_t seed)
{
  return ((seed >> 15) ^ ((seed >> 12) & 1)) | ((seed << 1) & 0xffff);
}

static inline void
update_noise (OPL * opl)
{
  opl->noise_seed = mrand (opl->noise_seed);
  whitenoise = opl->noise_seed & 1;
}

static inline void
update_ampm (OPL * opl)
{
  opl->pm_phase = (opl->pm_phase + pm_dphase) & (PM_DP_WIDTH - 1);
  opl->am_phase = (opl->am_phase + am_dphase) & (AM_DP_WIDTH - 1);
  lfo_am = amtable[opl->am_mode][HIGHBITS (opl->am_phase, AM_DP_BITS - AM_PG_BITS)];
  lfo_pm = pmtable[opl->pm_mode][HIGHBITS (opl->pm_phase, PM_DP_BITS - PM_PG_BITS)];
}

static inline uint32_t
calc_phase (OPL_SLOT * slot)
{
  if (slot->patch->PM)
    slot->phase += (slot->dphase * lfo_pm) >> PM_AMP_BITS;
  else
    slot->phase += slot->dphase;

  slot->phase &= (DP_WIDTH - 1);

  return HIGHBITS (slot->phase, DP_BASE_BITS);
}

static inline uint32_t
calc_envelope (OPL_SLOT * slot)
{
#define S2E(x) (ALIGN((int32_t)(x/SL_STEP),SL_STEP,EG_STEP)<<(EG_DP_BITS-EG_BITS))
  static uint32_t SL[16] = {
    S2E (0), S2E (3), S2E (6), S2E (9), S2E (12), S2E (15), S2E (18), S2E (21),
    S2E (24), S2E (27), S2E (30), S2E (33), S2E (36), S2E (39), S2E (42), S2E (93)
  };
  uint32_t egout;


  switch (slot->eg_mode)
  {

  case ATTACK:
    slot->eg_phase += slot->eg_dphase;
    if (EG_DP_WIDTH & slot->eg_phase)
    {
      egout = 0;
      slot->eg_phase = 0;
      slot->eg_mode = DECAY;
      UPDATE_EG (slot);
    }
    else
    {
      egout = AR_ADJUST_TABLE[HIGHBITS (slot->eg_phase, EG_DP_BITS - EG_BITS)];
    }
    break;

  case DECAY:
    slot->eg_phase += slot->eg_dphase;
    egout = HIGHBITS (slot->eg_phase, EG_DP_BITS - EG_BITS);
    if (slot->eg_phase >= SL[slot->patch->SL])
    {
      if (slot->patch->EG)
      {
        slot->eg_phase = SL[slot->patch->SL];
        slot->eg_mode = SUSHOLD;
        UPDATE_EG (slot);
      }
      else
      {
        slot->eg_phase = SL[slot->patch->SL];
        slot->eg_mode = SUSTINE;
        UPDATE_EG (slot);
      }
      egout = HIGHBITS (slot->eg_phase, EG_DP_BITS - EG_BITS);
    }
    break;

  case SUSHOLD:
    egout = HIGHBITS (slot->eg_phase, EG_DP_BITS - EG_BITS);
    if (slot->patch->EG == 0)
    {
      slot->eg_mode = SUSTINE;
      UPDATE_EG (slot);
    }
    break;

  case SUSTINE:
  case RELEASE:
    slot->eg_phase += slot->eg_dphase;
    egout = HIGHBITS (slot->eg_phase, EG_DP_BITS - EG_BITS);
    if (egout >= (1 << EG_BITS))
    {
      slot->eg_mode = FINISH;
      egout = (1 << EG_BITS) - 1;
    }
    break;

  case FINISH:
    egout = (1 << EG_BITS) - 1;
    break;

  default:
    egout = (1 << EG_BITS) - 1;
    break;
  }

  if (slot->patch->AM)
    egout = ALIGN (egout + slot->tll, EG_STEP, DB_STEP) + lfo_am;
  else
    egout = ALIGN (egout + slot->tll, EG_STEP, DB_STEP);

  if (egout >= DB_MUTE)
    egout = DB_MUTE - 1;

  return egout;
}

static inline int32_t
calc_slot_car (OPL_SLOT * slot, int32_t fm)
{

  slot->egout = calc_envelope (slot);
  slot->pgout = calc_phase (slot);
  if (slot->egout >= (DB_MUTE - 1))
    return 0;

  return DB2LIN_TABLE[slot->sintbl[(slot->pgout + wave2_8pi (fm)) & (PG_WIDTH - 1)] + slot->egout];

}

static inline int32_t
calc_slot_mod (OPL_SLOT * slot)
{

  int32_t fm;

  slot->output[1] = slot->output[0];
  slot->egout = calc_envelope (slot);
  slot->pgout = calc_phase (slot);

  if (slot->egout >= (DB_MUTE - 1))
  {
    slot->output[0] = 0;
  }
  else if (slot->patch->FB != 0)
  {
    fm = wave2_4pi (slot->feedback) >> (7 - slot->patch->FB);
    slot->output[0] = DB2LIN_TABLE[slot->sintbl[(slot->pgout + fm) & (PG_WIDTH - 1)] + slot->egout];
  }
  else
    slot->output[0] = DB2LIN_TABLE[slot->sintbl[slot->pgout] + slot->egout];

  slot->feedback = (slot->output[1] + slot->output[0]) >> 1;

  return slot->feedback;

}

static inline int16_t
mix_output(OPL * opl)
{
  int32_t out = opl->ch_out[0];
  int i;
  for(i=1;i<15;i++) {
    out+= opl->ch_out[i];
  }
  return (int16_t)out;
}

int16_t
OPL_calc (OPL * opl)
{

  int32_t inst = 0, perc = 0, out = 0;
  int32_t i;

  update_ampm (opl);
  update_noise (opl);

  for (i = 0; i < 6; i++)
  {
    if (!(opl->mask&OPL_MASK_CH(i)) && (opl->CAR (i)->eg_mode != FINISH))
    {
      if (opl->ch[i]->alg)
        opl->ch_out[i] += calc_slot_car (opl->CAR (i), 0) + calc_slot_mod (opl->MOD (i));
      else
        opl->ch_out[i] += calc_slot_car (opl->CAR (i), calc_slot_mod (opl->MOD (i)));
    }
    opl->ch_out[i] >>= 1;
  }

  if (!opl->rhythm_mode)
  {
    for (i = 6; i < 9; i++)
    {
      if (!(opl->mask&OPL_MASK_CH(i)) && (opl->CAR (i)->eg_mode != FINISH))
      {
        if (opl->ch[i]->alg)
          opl->ch_out[i] += calc_slot_car (opl->CAR (i), 0) + calc_slot_mod (opl->MOD (i));
        else
          opl->ch_out[i] += calc_slot_car (opl->CAR (i), calc_slot_mod (opl->MOD (i)));
      }
      opl->ch_out[i] >>= 1;
    }
  }

  out = inst + perc;
  if(!(opl->mask&OPL_MASK_PCM)) {
    opl->ch_out[14] += ADPCM_calc (opl->adpcm);
  }
  opl->ch_out[14] >>= 1;

  return mix_output(opl);

}

void
OPL_writeReg (OPL * opl, uint32_t reg, uint32_t data)
{

  int32_t s, c;

  int32_t stbl[32] = { 0, 2, 4, 1, 3, 5, -1, -1,
    6, 8, 10, 7, 9, 11, -1, -1,
    12, 14, 16, 13, 15, 17, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1
  };

  data = data & 0xff;
  reg = reg & 0xff;


  if ((0x07 <= reg) && (reg <= 0x12))
    ADPCM_writeReg (opl->adpcm, reg, data);
  else if ((0x20 <= reg) && (reg < 0x40))
  {
    s = stbl[(reg - 0x20)];
    if (s >= 0)
    {
      opl->slot[s]->patch->AM = (data >> 7) & 1;
      opl->slot[s]->patch->PM = (data >> 6) & 1;
      opl->slot[s]->patch->EG = (data >> 5) & 1;
      opl->slot[s]->patch->KR = (data >> 4) & 1;
      opl->slot[s]->patch->ML = (data) & 15;
      UPDATE_ALL (opl->slot[s]);
    }
  }
  else if ((0x40 <= reg) && (reg < 0x60))
  {

    s = stbl[(reg - 0x40)];
    if (s >= 0)
    {
      opl->slot[s]->patch->KL = (data >> 6) & 3;
      opl->slot[s]->patch->TL = (data) & 63;
      UPDATE_ALL (opl->slot[s]);
    }

  }
  else if ((0x60 <= reg) && (reg < 0x80))
  {

    s = stbl[(reg - 0x60)];
    if (s >= 0)
    {
      opl->slot[s]->patch->AR = (data >> 4) & 15;
      opl->slot[s]->patch->DR = (data) & 15;
      UPDATE_EG (opl->slot[s]);
    }

  }
  else if ((0x80 <= reg) && (reg < 0xA0))
  {
    s = stbl[(reg - 0x80)];
    if (s >= 0)
    {
      opl->slot[s]->patch->SL = (data >> 4) & 15;
      opl->slot[s]->patch->RR = (data) & 15;
      UPDATE_EG (opl->slot[s]);
    }
  }
  else if ((0xA0 <= reg) && (reg < 0xA9))
  {
    c = reg - 0xA0;
    if (c >= 0)
    {
      setFnumber (opl, c, data + ((opl->reg[reg + 0x10] & 3) << 8));
      UPDATE_ALL (opl->CAR (c));
      UPDATE_ALL (opl->MOD (c));
    }

  }
  else if ((0xB0 <= reg) && (reg < 0xB9))
  {

    c = reg - 0xB0;
    if (c >= 0)
    {
      setFnumber (opl, c, ((data & 3) << 8) + opl->reg[reg - 0x10]);
      setBlock (opl, c, (data >> 2) & 7);

      if (((opl->reg[reg] & 0x20) == 0) && (data & 0x20))
        keyOn (opl, c);
      else if ((data & 0x20) == 0)
        keyOff (opl, c);

      UPDATE_ALL (opl->MOD (c));
      UPDATE_ALL (opl->CAR (c));
    }

  }
  else if ((0xC0 <= reg) && (reg < 0xC9))
  {

    c = reg - 0xC0;
    if (c >= 0)
    {
      opl->slot[c * 2]->patch->FB = (data >> 1) & 7;
      opl->ch[c]->alg = data & 1;
    }

  }
  else if (reg == 0xbd)
  {
    opl->rhythm_mode = data & 32;
    opl->am_mode = (data >> 7) & 1;
    opl->pm_mode = (data >> 6) & 1;
  }

  opl->reg[reg] = (unsigned char) data;

}

uint32_t
OPL_readIO (OPL * opl)
{
  return opl->reg[opl->adr];
}

void
OPL_writeIO (OPL * opl, uint32_t adr, uint32_t val)
{
  adr &= 0xff;
  if (adr & 1)
    OPL_writeReg (opl, opl->adr, val);
  else
    opl->adr = val;
}

uint32_t
OPL_status (OPL * opl)
{
  return ADPCM_status (opl->adpcm);
}
