/*!
    \file    readme.txt
    \brief   description of dual CAN mode and dual CANFD mode communication demo

    \version 2025-08-26, V0.1.0, demo for GD32A7xx
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

  This demo is based on the GD32A712AI-KIT-LQFP176-V1.0 board, it shows how to configure the 
DTM_CAN4 peripheral to send CAN frames, and DTM_CAN5 peripheral to receive CAN frames in normal mode,
and how to configure the DTM_CAN0 peripheral to send CANFD frames, and DTM_CAN2 peripheral to receive
CANFD frames in normal mode.

  When K1 button is pressed, DTM_CAN4 sends a CAN message to DTM_CAN5, DTM_CAN0 sends a CANFD message to DTM_CAN2.
When no available Tx buffers for DTM_CAN4 transmission, LED1 will be on. When no available Tx buffers for DTM_CAN0 
transmission, LED2 will be on. When DTM_CAN5 receives correctly, the receive data will be printed and LED3 toggles 
one time. When DTM_CAN2 receives correctly,the receive data will be printed and LED4 toggles one time.

  User must connect L pin to L pin, and H pin to H pin of JP8 and JP9, and connect L pin to L pin, 
and H pin to H pin of CN3 and CN4 on the board for sending and receiving frames.
