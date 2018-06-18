// General includes
#include <stdlib.h>
#include <print.h>
#include <time.h>
#include <math.h>
#include <stdio.h>

// SSRC includes
#include "src.h"
#include "dsp_wrapper.h" //Helps call C functions from XC

//UAC2 includes
#include "devicedefines.h"

static void bomb_out(void){
    printstrln("FATAL - SSRC procedure returned error");
}

// SSRC instances variables - State, Stack, Coefs and Control structures
//static SSRCState_t     sSSRCState[SSRC_N_CHANNELS];
//static SSRCCtrl_t      sSSRCCtrl[SSRC_N_CHANNELS];
static ssrc_state_t    sSSRCState[SSRC_N_CHANNELS];
static int             iSSRCStack[SSRC_N_CHANNELS][SSRC_STACK_LENGTH_MULT * SSRC_N_IN_SAMPLES];
static ssrc_ctrl_t     sSSRCCtrl[SSRC_N_CHANNELS];

//Initialise each instance
void dsp_init(unsigned sr_in, unsigned sr_out, unsigned thread_id)
{

    //for(int i=0; i<NUM_CHANNELS_PER_SSRC; i++)

    unsigned instance = thread_id;

    // Set state, stack and coefs into ctrl structure
    sSSRCCtrl[instance].psState                   = &sSSRCState[instance];
    sSSRCCtrl[instance].piStack                   = iSSRCStack[instance];

    // Set number of samples
    sSSRCCtrl[instance].uiNInSamples              = SSRC_N_IN_SAMPLES;

    // Set number of channels
    sSSRCCtrl[instance].uiNchannels               = SSRC_N_IO_CHANNELS;

    // Set dither flag and random seeds
    sSSRCCtrl[instance].uiDitherOnOff             = SSRC_DITHER_OFF;
    sSSRCCtrl[instance].uiRndSeedInit             = 1234567;


    sSSRCCtrl[instance].eInFs                     = sr_in;
    sSSRCCtrl[instance].eOutFs                    = sr_out;

    // Init SSRC instances
    if(SSRC_init(&sSSRCCtrl[instance]) != SSRC_NO_ERROR) bomb_out();
    // Sync (Ie. clear delay line)
    //if(SSRC_sync(&sSSRCCtrl[instance]) != SSRC_NO_ERROR) bomb_out(); Removed to work around DAC noise on ADC SR change
}

//Call the DSP processing function
unsigned dsp_process(int *in_buff, int *out_buff, unsigned thread_id){


    unsigned instance = thread_id;

    //for(int i=0; i<NUM_CHANNELS_PER_SSRC; i++)

    sSSRCCtrl[instance].piIn  = (int *) in_buff;    //Setup input buffer
    sSSRCCtrl[instance].piOut = (int *) out_buff;   //Setup output buffer
    if(SSRC_proc(&sSSRCCtrl[instance]) != SSRC_NO_ERROR) bomb_out();
    unsigned n_samps_out = (*sSSRCCtrl[instance].puiNOutSamples);   //Get number of samples returned
    return n_samps_out;
}

