/*
 * dsp_wrapper.h
 *
 *  Created on: Jul 17, 2015
 *      Author: Ed
 */
#include "customdefines.h"

//Helps call C stuff from XC

#ifndef DSP_WRAPPER_H_
#define DSP_WRAPPER_H_

#define         SSRC_N_CHANNELS                 NUM_USB_CHAN_OUT    //Number of SSRC channels overall
#define         NUM_CHANNELS_PER_SSRC           1                   //SSRC channels per thread
#define         SSRC_N_IO_CHANNELS              NUM_CHANNELS_PER_SSRC//SHould be set to SSRC channels per thread
#define         SSRC_N_IN_SAMPLES               4                   //Samples provided to input per SSRC call. Minimum 4
#define         SSRC_N_IN_OUT_RATIO_MAX         5                   //Max number of samples out per sample in, 44.1->192 = 5
#define         MAX_NUM_SSRC_SAMPS_OUT          (SSRC_N_IN_OUT_RATIO_MAX * SSRC_N_IN_SAMPLES)
#define         OUT_BUFF_SIZE                   (MAX_NUM_SSRC_SAMPS_OUT * 8) //Size of FIFO on output of SSRC
#define         SSRC_THREADS                    (SSRC_N_CHANNELS / NUM_CHANNELS_PER_SSRC) //Number of DSP engine threads

#ifdef __XC__
void dsp_init(unsigned sr_in, unsigned sr_out, unsigned instance);
unsigned dsp_process(int in_buff[], int out_buff[], unsigned instance);
#else
void dsp_init(unsigned sr_in, unsigned sr_out, unsigned instance);
unsigned dsp_process(int *in_buff, int *out_buff, unsigned instance);
#endif

#endif /* DSP_WRAPPER_H_ */
