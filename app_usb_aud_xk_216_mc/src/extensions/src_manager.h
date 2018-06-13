/*
 * src_manager.h
 *
 *  Created on: Oct 28, 2015
 *      Author: Ed
 */


#ifndef SRC_MANAGER_H_
#define SRC_MANAGER_H_

/**
void src_manager(chanend c_host, chanend c_i2s, chanend c_i2s_sampfreq)
*
* Top level src manager task called from main, i.e. starts some parallell tasks.
*     - manager, src_manager_task - sits between USB/Decouple and Audio/I2S 
*       which manages different rates and farms out samples to the DSP engines. 
*       It is transparent to both the decouple and audio tasks. It does this by 
*       emulating the channel protocol presented by these two tasks.
*     - DSP-engine, dsp_task(s) - receives/sends samples and calls SSRC
*
* @param c_host channel for audio in from host
* 
* @param c_i2s channel for audio out 
*
* @param c_i2s_sampfreq
* 
*/

void src_manager(chanend c_host, chanend c_i2s, chanend c_i2s_sampfreq);

#endif /* SRC_MANAGER_H_ */
