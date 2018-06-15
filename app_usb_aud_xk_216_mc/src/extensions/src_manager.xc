/*
 * src_manager.xc
 *
 *  Created on: Oct 28, 2015
 *      Author: Ed
 */
#include <xs1.h>
#include <print.h>
#include <string.h>
#include <stdio.h>
#include <timer.h>
#include <platform.h>
#include <xscope.h>
#include "xc_ptr.h"
#include "commands.h"
#include "src_manager.h"
#include "devicedefines.h"
#include "dsp_wrapper.h"

on tile[0]: out port p_busy         = XS1_PORT_1E;    //SPDIF OPTICAL. Used to measure performance
on tile[0]: in port p_buttons       = XS1_PORT_4D;    //buttons 1..3 and switch. Just button 1 used here
on tile[1]: in port p_buttons_1     = XS1_PORT_4B;    //Ensures this port is off and high Z (shared with P4D/tile 0)

//Buffers for input/output to SSRC
int g_samps_from_host[SSRC_THREADS][SSRC_N_IN_SAMPLES][NUM_CHANNELS_PER_SSRC];     //Single buffers for input
int g_samps_to_i2s[NUM_USB_CHAN_OUT][OUT_BUFF_SIZE];            //Circular buffers for output

//Output circular buffer pointers
unsafe{
    int * unsafe ptr_base_samps_to_i2s[NUM_USB_CHAN_OUT];
    int * unsafe ptr_rd_samps_to_i2s[NUM_USB_CHAN_OUT];
    int * unsafe ptr_wr_samps_to_i2s[NUM_USB_CHAN_OUT];
}


//Helper function for converting SR to index value
static unsigned samp_rate_to_code(unsigned samp_rate){
    unsigned samp_code = 0;
    switch (samp_rate){
    case 44100:
        samp_code = 0;
        break;
    case 48000:
        samp_code = 1;
        break;
    case 88200:
        samp_code = 2;
        break;
    case 96000:
        samp_code = 3;
        break;
    case 176400:
        samp_code = 4;
        break;
    case 192000:
        samp_code = 5;
        break;
    default:
        printstrln("Unsupported sample rate");
        break;
    }
    return samp_code;
}

//Helper function to work out the exact ratio of output samples per n_samples in
static unsigned calc_n_samples(unsigned samp_rate_code, unsigned n_samps_in){
    const unsigned code_to_samp_ratio[6] = {147, 160, 294, 320, 588, 640};
    unsigned n_samps_out;
    static unsigned remainder = 0;
    static unsigned numerator, divisor;
    static unsigned samp_rate_code_old = (0x12345678);  //Initialise to invalid code to force init on first loop

    if (samp_rate_code != samp_rate_code_old){
        divisor = code_to_samp_ratio[samp_rate_code & 0xffff];  //Upper word is SR in
        numerator = code_to_samp_ratio[samp_rate_code >> 16];   //Lower word is SR out
        remainder = divisor / 2;                                //Round to nearest(ish)
        samp_rate_code_old = samp_rate_code;
        //printf("New sample rate in=%d, out=%d\n", samp_rate_code >> 16, samp_rate_code & 0xffff);
    }

    n_samps_out = (n_samps_in * numerator + remainder) / divisor; //Whole number of samples
    remainder =  (n_samps_in * numerator + remainder) % divisor;  //Remainder to rollover to carry error

    return n_samps_out;
}

//Set FIFOs pointers to half and clear contents
static inline void init_fifos(){
    unsafe{
        for (int i=0 ; i<NUM_USB_CHAN_OUT; i++){
            ptr_base_samps_to_i2s[i] = &g_samps_to_i2s[i][0];
            ptr_rd_samps_to_i2s[i]   = ptr_base_samps_to_i2s[i];
            ptr_wr_samps_to_i2s[i]   = (ptr_base_samps_to_i2s[i] + OUT_BUFF_SIZE / 2);
            memset(ptr_base_samps_to_i2s[i], 0, OUT_BUFF_SIZE * sizeof(int));
        }
    }
}

//Used by producer to push samples from SSRC output into FIFO
static inline unsigned push_single_sample_to_fifo(int samp, int * unsafe * unsafe wr_ptr, int * unsafe rd_ptr, int * unsafe fifo_base){
    unsigned success = 1;
    unsafe{
        **wr_ptr = samp;    //write value into fifo
        (*wr_ptr)++;        //increment write pointer (by reference)
        if (*wr_ptr >= fifo_base + OUT_BUFF_SIZE) *wr_ptr = fifo_base; //wrap pointer
        if (*wr_ptr == rd_ptr) {
            success = 0;
        }
    }
    return success;
}

//Used by consumer of samples to pull from output FIFO from SSRC
static inline unsigned pull_sample_from_fifo(int &samp, int * unsafe wr_ptr, int * unsafe  * unsafe rd_ptr, int * unsafe fifo_base){
    unsigned success = 1;
    unsafe{
        samp = **rd_ptr;    //read value from fifo
        (*rd_ptr)++; //increment write pointer (by reference)
        if (*rd_ptr >= fifo_base + OUT_BUFF_SIZE) *rd_ptr = fifo_base; //wrap pointer
        if (*rd_ptr == wr_ptr){
            success = 0;
        }
    }
    return success;
}


//Function to send and receive samples to DSP engines
#pragma unsafe arrays   //Need to remove bounds check to meet timing at 192KHz
static inline unsigned trigger_dsp(streaming chanend c_dsp[SSRC_THREADS],
        int samps_from_host[SSRC_THREADS][SSRC_N_IN_SAMPLES][NUM_CHANNELS_PER_SSRC], unsigned sample_rate_code)
{
    unsigned pull_count;
#pragma loop unroll
    for (int i=0; i<SSRC_THREADS; i++) c_dsp[i] <: sample_rate_code;//Send sample rate control token initially

#pragma loop unroll
    for (int i=0; i<SSRC_THREADS; i++){                             //Send frame of samples out to DSP engine
#pragma loop unroll
        for (int j=0; j<SSRC_N_IN_SAMPLES; j++){                    //Send to each engine
#pragma loop unroll
            for (int k=0; k<NUM_CHANNELS_PER_SSRC; k++){            //Send number of samples handled by each engine
                c_dsp[i] <: samps_from_host[i][j][k];
            }
        }
    }

#pragma loop unroll
    for (int i=0; i<SSRC_THREADS; i++) c_dsp[i] :> pull_count;      //Get number of samples to receive

#pragma loop unroll
    for (int i=0; i<SSRC_THREADS; i++){                             //From each engine..
        unsigned success = 1;
        for (int j=0; j<pull_count; j++){                           //Get n samples
#pragma loop unroll
            for (int k=0; k<NUM_CHANNELS_PER_SSRC; k++){            //..for each channel handled by engine
                int samp;
                c_dsp[i] :> samp;
                if (i==0) xscope_int(TO_I2S_LEFT, samp);
                if (i==1) xscope_int(TO_I2S_RIGHT, samp);
                unsigned channel_no = k + i * NUM_CHANNELS_PER_SSRC;
                unsafe{                                                 //And push them into the FIFO
                    success &= push_single_sample_to_fifo(samp,
                            &ptr_wr_samps_to_i2s[channel_no],
                            ptr_rd_samps_to_i2s[channel_no],
                            ptr_base_samps_to_i2s[channel_no]);
                }
            }//channels
        }//instances
        if (!success) {
            //printchar('f');
            init_fifos();
        }
    } //pull_count
    return pull_count;
}

//This is the main task that sits between USB/Decouple and Audio/I2S which manages different rates and
//and farms out samples to the DSP engines. It is transparent to both the decouple and audio tasks.
//It does this by emulating the channel protocol presented by these two tasks.
#pragma unsafe arrays
void src_manager_task(chanend c_host, chanend c_i2s, streaming chanend c_dsp[SSRC_THREADS], chanend c_i2s_sampfreq)
{
    unsigned request;           //Request code from I2S. Part of ref design protocol
    unsigned samples_idx = 0;   //Index for counting SSRC_N_IN_SAMPLES sets of samples
    unsigned control_flag = 0;  //Control request from host. Will trigger forwarding to I2S
    unsigned command[3];        //Control request packet from host - up to 3 words
    unsigned sr_host = DEFAULT_FREQ, sr_i2s = DEFAULT_FREQ;   //Sample rates
    unsigned n_samps = 1;       //Number of samples to pull
    unsigned samp_rate_code =  samp_rate_to_code(DEFAULT_FREQ) << 16 | samp_rate_to_code(DEFAULT_FREQ);
    unsigned buttons_last, buttons_now; //Buttons value read from port


    timer t;                    //Debug timing
    int t0, t1, t2, t3;         //Debug timing

    init_fifos();
    p_buttons :> buttons_last;

    printstrln("src_manager started");
    while (1)
    {
        ////////////////////Send single sample to I2S////////////////
        request = inuint(c_i2s);        //Receive request from audio()
        p_busy <: 1;
        t :> t2;
        if (control_flag){              //SR or stream change flag from host transaction
            outct(c_i2s, command[0]);   //Send command

            /*  SET_SAMPLE_FREQ         4
                SET_STREAM_FORMAT_OUT   8
                SET_STREAM_FORMAT_IN    9 */
            printf("Received SR or stream change\n");
            switch(command[0])
            {
                case SET_SAMPLE_FREQ:
                    printf("Received request SET_SAMPLE_FREQ from audio (c_i2s)\n");
                    printf("I2S running at SR=%d\n", sr_i2s);
                    printf("Host running at SR=%d\n", sr_host);
                    outuint(c_i2s, sr_i2s);
                    printf("Sent to audio (c_i2s): %d\n", sr_i2s);
                    break;

                case SET_STREAM_FORMAT_OUT:
                    printf("Received request SET_STREAM_FORMAT_OUT from audio (c_i2s)\n");
                case SET_STREAM_FORMAT_IN:
                    outuint(c_i2s, command[1]);
                    outuint(c_i2s, command[2]);
                    printf("Sent to audio (c_i2s): %d\n", command[1]);
                    printf("Sent to audio (c_i2s): %d\n", command[2]);
                    break;

                default:
                    break;
            }
            chkct(c_i2s, XS1_CT_END);   //Get handshake
            control_flag = 0;           //Clear control request flag
            init_fifos();
        }

        else{                       //Normal data transfer
            unsigned success = 1;   //FIFO read/write status
#pragma loop unroll
            for (int i=0; i<NUM_USB_CHAN_OUT; i++){  //Send samples to i2s
                int samp;
                unsafe{                              //pull samples from FIFO
                    success &= pull_sample_from_fifo(samp, ptr_wr_samps_to_i2s[i], &ptr_rd_samps_to_i2s[i], ptr_base_samps_to_i2s[i]);
                }
                if (i==0) xscope_int(TO_I2S, samp); //bak
                //if (i==0) xscope_int(-1, samp);

                if(success) {
                    outuint(c_i2s, samp);  //Send sample to i2s
                    /*if (samp) {
                        printf("Sent one sample to audio: %d\n", samp);
                    }*/
                }
                else        outuint(c_i2s, 0);     //Mute if buffer under/over-run
            }
            if(!success){   //Only reset FIFO when whole packet transferred to keep FIFO pointers properly aligned
                printchar('F');
                init_fifos();
            }
        }

        ////////////////Calculate n samps to get from host///////////////
        n_samps = calc_n_samples(samp_rate_code, 1);

        ////////////////////Get n samps to get from host/////////////////
        for(int i = 0; i<n_samps; i++){
            outuint(c_host, request);       //Send request for data to decouple thread

            if(testct(c_host))              //Check for control transaction
            {
                control_flag = 1;           //Signal that we have a control request pending for i2s
                i = n_samps;                //Break for loop
                command[0] = inct(c_host);
                switch(command[0])
                {
                    case SET_SAMPLE_FREQ:
                        command[1] = inuint(c_host);
                        sr_host = command[1];
                        samp_rate_code = (samp_rate_to_code(sr_host) << 16) | samp_rate_to_code(sr_i2s);
                        c_i2s_sampfreq <: sr_i2s; //Send sample rate to buffer for correct MCLK calc
                        printf("Received SET_SAMPLE_FREQ from host. Setting to sr %d\n", sr_host);
                        break;

                    case SET_STREAM_FORMAT_OUT:
                    case SET_STREAM_FORMAT_IN:
                        command[1] = inuint(c_host); //Get command packet
                        command[2] = inuint(c_host);
                        printf("Received SET_STREAM_FORMAT_OUT/IN from host.\n");
                        printf("Recv from host (c_host): %d\n", command[1]);
                        printf("Recv from host (c_host): %d\n", command[2]);
                        break;

                    default:
                        break;
                }
                outct(c_host, XS1_CT_END);  // Send handshake
            }
            else{   //Get single sample for each channel
#pragma loop unroll
                for (int chan_i=0; chan_i<NUM_USB_CHAN_OUT; chan_i++){  //Get samples from host
                    int samp = inuint(c_host);
                    if (chan_i==0) xscope_int(FROM_DECOUPLE_LEFT, samp);  //bak
                    //if (chan_i==0) xscope_int(-1, samp);
                    g_samps_from_host[chan_i/NUM_CHANNELS_PER_SSRC]
                                     [samples_idx]
                                     [chan_i % NUM_CHANNELS_PER_SSRC] = samp;
                }
            }

            ////////////////////Call SSRC here when we have SSRC_N_IN_SAMPLES samples////////////////
            samples_idx++;
            if (samples_idx == SSRC_N_IN_SAMPLES){
                samples_idx = 0;
                t :> t0;
                unsigned n_samps_from_dsp = trigger_dsp(c_dsp, g_samps_from_host, samp_rate_code);
                t :> t1;
                //printf("trigger_dsp triggered.\n");
            }
        } //for loops n_samps

        //Poll for button press to change i2s frequency. We get a free debounce effect from the loop timing
        p_buttons :> buttons_now;
        buttons_now &= 0x01;    //Mask out all but button 1
        if ((buttons_now != buttons_last) && (buttons_now == 0)) {
            switch(sr_i2s){
            case 44100:
                sr_i2s = 48000;
            break;
            case 48000:
                sr_i2s = 88200;
            break;
            case 88200:
                sr_i2s = 96000;
            break;
            case 96000:
                sr_i2s = 176400;
            break;
            case 176400:
                sr_i2s = 192000;
            break;
            case 192000:
                sr_i2s = 44100;
            break;
            default:
                sr_i2s = 44100;
            break;
            }
            command[0] = SET_SAMPLE_FREQ;   //Insert command to audio to change sample rate
            samp_rate_code = (samp_rate_to_code(sr_host) << 16) | samp_rate_to_code(sr_i2s);
            c_i2s_sampfreq <: sr_i2s;       //Send sample rate to buffer for correct MCLK calc
            control_flag = 1;               //Signal that we have a control request pending for i2s
        }
        buttons_last = buttons_now;

        p_busy <: 0;
        t :> t3;

        //Print time taken to trigger dsp. Do this every few seconds to avoid spamming the console
        //static int counter; counter++; if (counter > 54321){ printintln(t1-t0); counter = 0;}

    } //while (1)
}

//The DSP engine tasks that receives/sends samples and calls SSRC.
//There may be multiple instances of this function accoring to the value of SSRC_THREADS. See par{} below
void dsp_task(streaming chanend c_dsp, unsigned instance_id)
{
    unsigned int    sr_in_out = 99999; //Invalid SR code to force initialisation
    unsigned int    sr_in_out_new;

    int        in_buff[SSRC_N_IN_SAMPLES * NUM_CHANNELS_PER_SSRC];
    int        out_buff[SSRC_N_IN_SAMPLES * SSRC_N_IN_OUT_RATIO_MAX * NUM_CHANNELS_PER_SSRC];

    timer t;                            //debug timing
    int t1=0, t2=0, t_tot=0;
    unsigned count = instance_id * 100; //give it an offset so they don't print at the same time

    unsigned int    n_samps_out = 0;
    unsigned int    n_samps_out_old = 0;
    unsigned int    n_samps_in = 0;

    memset(out_buff, 0, SSRC_N_IN_SAMPLES * SSRC_N_IN_OUT_RATIO_MAX * NUM_CHANNELS_PER_SSRC * 4);
    //printstrln("DSP_task");

    while(1){

        t_tot = (t2 - t1);
        //count++; if (count>50000){count=0; printintln(t_tot);}
        c_dsp :> sr_in_out_new;
        n_samps_in += SSRC_N_IN_SAMPLES;

        for(unsigned i=0; i<SSRC_N_IN_SAMPLES * NUM_CHANNELS_PER_SSRC; i++) {
            int tmp;
            c_dsp :> tmp;
            in_buff[i] = tmp;
        }

        n_samps_in += SSRC_N_IN_SAMPLES;
        c_dsp <: n_samps_out;

        for(unsigned uj = 0; uj < n_samps_out * NUM_CHANNELS_PER_SSRC; uj++)
        {
            int tmp;
            tmp = out_buff[uj];
            c_dsp <: tmp;
        }

        if (sr_in_out_new != sr_in_out) {
            // Set input/output sampling rate codes. See SSRCFs_t in SSRC.h (ie 0=44.1 5=192)
            unsigned InFs                     = (sr_in_out_new >> 16) & 0xffff;
            unsigned OutFs                    = sr_in_out_new & 0xffff;

            dsp_init(InFs, OutFs, instance_id);
            sr_in_out = sr_in_out_new;

            /* Code  FS
             * 0     44100
             * 1     48000
             * 2     88200
             * 3     96000
             * 4    176400
             * 5    192000*/
            printf("Call to dsp_init performed. InstanceId = %d, InFs = %d, OutFs = %d\n", instance_id, InFs, OutFs);
        }
        t:> t1;
        //printf("DSP in instance %d do dsp\n", instance_id);
        n_samps_out = dsp_process(in_buff, out_buff, instance_id);
        t :> t2;
        xscope_int(FROM_DECOUPLE_DSP_TASK, in_buff[0]);  //From decouple dsp task
        xscope_int(FROM_DSP_DSP_TASK, out_buff[0]);      //From dsp dsp task
    }
}


//Top level src manager task called from main. Calls manager and engines
void src_manager(chanend c_host, chanend c_i2s, chanend c_i2s_sampfreq){
    streaming chan c_dsp[SSRC_THREADS];
    par
    {
        src_manager_task(c_host, c_i2s, c_dsp, c_i2s_sampfreq);         //Buffer manager
        par (int i=0; i < SSRC_THREADS; i++) dsp_task(c_dsp[i], i);     //DSP Engines
    }
}
