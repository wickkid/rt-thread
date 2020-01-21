/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2014-04-27     Bernard      make code cleanup.
 * 2017-08-25     LongfeiMa    transplantation for stm32h7xx
 */
#include <stdio.h>
#include <rtdevice.h>
#include "drv_sdram.h"
#include "drv_adc.h"
#include <udp_api.h>
#define Bank5_SDRAM_ADDR EXT_SDRAM_BEGIN
static void check_extern_sdram(void){//check sdram
		volatile rt_uint32_t i=0;  	  
		volatile rt_uint32_t temp=0;	   
		volatile rt_uint32_t deta=0;	   
		volatile rt_uint32_t sval=0;	//在地址0读到的数据	  
		rt_kprintf("Scan SDRAM Capacty\n");
		for(i=0;i<32*1024*1024;i+=16*1024)
		{
			*(volatile rt_uint32_t *)(Bank5_SDRAM_ADDR+i)=temp; 
			temp++;
		}
		//依次读出之前写入的数据,进行校验		  
		for(i=0;i<32*1024*1024;i+=16*1024) 
		{	
			temp=*(volatile rt_uint32_t*)(Bank5_SDRAM_ADDR+i);
			//rt_kprintf("S%u %08X\n", deta, temp);
			if(i==0){
				sval=temp;
				if (sval != 0){
					rt_kprintf("SDRAM read first 4B error\n");
					break;
				}			
			}
			else if(temp<=sval)break;//后面读出的数据一定要比第一次读到的数据大.	 
			else if (deta != (temp - sval)){
					rt_kprintf("SDRAM read S%u 16K error\n",deta);
					break;
			}
			deta++;
		}				
		temp = (rt_uint16_t)(temp-sval+1)*16;
		if (temp %1024)
			rt_kprintf("S%u SDRAM Capacity:%dKB\r\n",deta, temp);//打印SDRAM容量
		else
			rt_kprintf("SDRAM Capacity:%dMB\r\n", temp/1024);//打印SDRAM容量
	}
int main(void)
{
	#ifdef RT_USING_RUDP
	udp_api_handle_t udp_api_hd  = RT_NULL;
#endif
	check_extern_sdram();
	
#ifdef RT_USING_RUDP
	udp_api_hd = udp_api_create(60000, "192.168.3.199", 60002, "udp");
#endif
#ifdef RT_USING_ADC
	{//check adc normal
		rt_uint32_t values[4];
		rt_uint8_t cnt = 10;
		rt_adc_device_t devadc = (rt_adc_device_t)rt_device_find("adc");
		if (!devadc){
			rt_kprintf("adc is not found\n");
			return -1;
		}
		rt_adc_enable(devadc, 0);
		rt_kprintf("\n");
		while (cnt-- > 0){
			for(rt_uint8_t n = 0; n < 4; n++){			
				values[n] = rt_adc_read(devadc, n );
				rt_thread_delay(1);
			}
			rt_kprintf("value : %d\t %d\t %d\t %d\n", values[0], values[1], values[2], values[3]);
		}
	}
#elif defined(RT_USING_ADCONVERT)
		{
			rt_device_t devadc = rt_device_find("adc");
			
			rt_uint32_t offs = 0, offs_bak = ADC_HALF_BUFF_SZ;
			static rt_tick_t tick, dtick, st;
			
			rt_uint16_t* sa = (rt_uint16_t*)ADC_CNLA;
			rt_uint16_t* sb = (rt_uint16_t*)ADC_CNLB;
			rt_uint16_t* sc = (rt_uint16_t*)ADC_CNLC;
			//rt_uint16_t* sv = sc + 1;			
			rt_uint16_t* da = (rt_uint16_t*)(0xC1000000);
			rt_uint16_t* db = (rt_uint16_t*)(0xC1000000 + ADC_BUFF_SZ*2);
			rt_uint16_t* dc = (rt_uint16_t*)(0xC1000000 + ADC_BUFF_SZ * 4);
			/*rt_uint16_t* dv = (rt_uint16_t*)(0xC1000000 + ADC_BUFF_SZ * 3);*/
			rt_memset((void*)EXT_SDRAM_BEGIN, 0xAA, EXT_SDRAM_SIZE);

			if (devadc){
				tick = rt_tick_get();
				if (RT_EOK == rt_device_open(devadc,0)){	
					while(1){
						rt_device_control(devadc, RT_DEVICE_ADC_WAIT_OFFSET, &offs);
						dtick = rt_tick_get() - tick;
						tick += dtick;
						if (offs == offs_bak)
							rt_kprintf("cost %d %d\n", dtick, st);
						RT_ASSERT (offs != offs_bak);
						offs_bak = offs;
						
						st = rt_tick_get();
						udp_api_send(udp_api_hd, (rt_uint8_t*)ADC_CNLA, ADC_BUFF_SZ * 3/*11000 ADC_BUFF_SZ  * 3*/);//ADC_BUFF_SZ  1000000
						st = rt_tick_get() - st;
						
						/*
						for (uint32_t n = offs; n < ADC_HALF_BUFF_SZ; n++){
							*da++ = *sa++;
							*db++ = *sb++;
							*dc++ = *sc++;
							/**dv++ = *sc++;		* /					
						}*/
						//rt_thread_delay(1);
					}
					rt_device_close(devadc);
				}
			}			
		}
#endif
		
		while(1) rt_thread_delay(RT_TICK_PER_SECOND);
		

	return 0;
}

