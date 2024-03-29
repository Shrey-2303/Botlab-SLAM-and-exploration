/*!
  * Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
  * 
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  * 
  * Redistributions of source code must retain the above copyright
  * notice, this list of conditions and the following disclaimer.
  * 
  * Redistributions in binary form must reproduce the above copyright
  * notice, this list of conditions and the following disclaimer in the
  * documentation and/or other materials provided with the distribution.
  * 
  * Neither the name of the copyright holder nor the names of the
  * contributors may be used to endorse or promote products derived from
  * this software without specific prior written permission.
  * 
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
  * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
  * OR CONTRIBUTORS BE LIABLE FOR ANY
  * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
  * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  * ANY WAY OUT OF THE USE OF THIS
  * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
  * 
  * The information provided is believed to be accurate and reliable.
  * The copyright holder assumes no responsibility
  * for the consequences of use
  * of such information nor for any infringement of patents or
  * other rights of third parties which may result from its use.
  * No license is granted by implication or otherwise under any patent or
  * patent rights of the copyright holder.
  *
  * @file          bhy_uc_driver_config.h
  *
  * @date          12/15/2016
  *
  * @brief         header file of bhy_uc_driver.c
  *
  */




#ifndef BHY_UC_DRIVER_CONFIG_H_
#define BHY_UC_DRIVER_CONFIG_H_

/****************************************************************************/
/*                      Driver configuration                                */
/****************************************************************************/
#define BHY_MCU_REFERENCE_VERSION  "1.1.1.0"

/* Enabling BHY_DEBUG will allow you print all the sensor raw data */
//#define BHY_DEBUG

/* Enabling BHY_CALLBACK_MODE will allow you to install callback function   */
/* that will be called from the bhy_parse_next_fifo_packet function. is uses*/
/* ~250 bytes of ROM and ~350 bytes of RAM                                  */
#define BHY_CALLBACK_MODE 1


#endif /* BHY_UC_DRIVER_CONFIG_H_ */
