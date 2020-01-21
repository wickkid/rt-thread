#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "drv_gpio.h"

#define GPIO_PIN1_CLOCK_ENABLE()	__GPIOC_CLK_ENABLE()
#define GPIO_PIN1_PORT						GPIOC
#define GPIO_PIN1_PIN							GPIO_PIN_8
#define GPIO_PIN1_IRQ							EXTI9_5_IRQn


#define GPIO_PIN2_CLOCK_ENABLE()	__GPIOC_CLK_ENABLE()
#define GPIO_PIN2_PORT						GPIOC
#define GPIO_PIN2_PIN							GPIO_PIN_9
#define GPIO_PIN2_IRQ							EXTI9_5_IRQn

#define GPIO_PIN3_CLOCK_ENABLE()	__GPIOC_CLK_ENABLE()
#define GPIO_PIN3_PORT						GPIOC
#define GPIO_PIN3_PIN							GPIO_PIN_6
#define GPIO_PIN3_IRQ							EXTI9_5_IRQn

#define RISING_EDGE           ((uint32_t)0x00100000)
#define FALLING_EDGE          ((uint32_t)0x00200000)


#define GPIO_CLK_ENABLE() do{GPIO_PIN1_CLOCK_ENABLE();GPIO_PIN2_CLOCK_ENABLE();GPIO_PIN3_CLOCK_ENABLE();}while(0)

#define IRQ_MAX_VAL 3
#define PIN_MAX_VAL 3

#ifdef RT_USING_GPIO
#include "stm32h7xx.h"
struct rt_pin_irq_hdr pin_irq_hdr_tab[] =
{
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
};


static rt_uint16_t get_irq_unmask(IRQn_Type irq_type){
	if (irq_type == EXTI9_5_IRQn){
		return EXTI_D1->IMR1 & (GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);
	}
	
	return 0;
}




/*
static void pin_irq_hdr(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    int irqno = 0;
    for(irqno = 0; irqno < IRQ_MAX_VAL; irqno ++)
    {
        if((irqno) == pintr)
        {
            break;
        }
    }
    
    if(irqno >= IRQ_MAX_VAL)
        return;
    
    if (pin_irq_hdr_tab[irqno].hdr)
    {
        pin_irq_hdr_tab[irqno].hdr(pin_irq_hdr_tab[irqno].args);
    }
}*/
static void stm_pin_mode(rt_device_t dev, rt_base_t pin, rt_base_t mode)
{
    GPIO_TypeDef* portx;
		int piny, dir;
    uint32_t pin_cfg;
    
    if(pin > PIN_MAX_VAL)
        return;
    
		switch(pin)
		{
				case 0: portx = GPIO_PIN1_PORT; piny = GPIO_PIN1_PIN; break;
				case 1: portx = GPIO_PIN2_PORT; piny = GPIO_PIN2_PIN; break;
				case 2: portx = GPIO_PIN3_PORT; piny = GPIO_PIN3_PIN; break;
				default:break;
		} 
    
    switch(mode)
    {
    case PIN_MODE_OUTPUT: 
        dir = GPIO_MODE_OUTPUT_PP;
        pin_cfg = GPIO_NOPULL;
        break;
    case PIN_MODE_OUTPUT_OD:
        dir = GPIO_MODE_OUTPUT_OD;        
        pin_cfg = GPIO_NOPULL;        
        break;
    
    case PIN_MODE_INPUT:   
        dir = GPIO_MODE_INPUT;
        pin_cfg = GPIO_NOPULL;
        break;        
    case PIN_MODE_INPUT_PULLUP:
        dir = GPIO_MODE_INPUT;        
        pin_cfg = GPIO_PULLUP;
        break;
    case PIN_MODE_INPUT_PULLDOWN: 
        dir = GPIO_MODE_INPUT;
        pin_cfg = GPIO_PULLDOWN;        
        break;
    default: break;
    }      
    {
			GPIO_InitTypeDef  GPIO_InitStruct;
			GPIO_InitStruct.Mode      = dir;
			GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Pull      = pin_cfg;

			GPIO_InitStruct.Pin 			= piny;
			HAL_GPIO_Init(portx, &GPIO_InitStruct);	

		}

}
static void stm_pin_write(rt_device_t dev, rt_base_t pin, rt_base_t value)
{
    GPIO_TypeDef* portx;
		int piny;
    if(pin > PIN_MAX_VAL)
       return;		
		
		switch(pin)
		{
				case 0: portx = GPIO_PIN1_PORT; piny = GPIO_PIN1_PIN; break;
				case 1: portx = GPIO_PIN2_PORT; piny = GPIO_PIN2_PIN; break;
				case 2: portx = GPIO_PIN3_PORT; piny = GPIO_PIN3_PIN; break;
				default:break;
		}

    HAL_GPIO_WritePin(portx, piny, (GPIO_PinState)value);        
}

static int stm_pin_read(rt_device_t dev, rt_base_t pin)
{
    GPIO_TypeDef* portx;
		int piny, value;    
    
    if(pin > PIN_MAX_VAL)
        return RT_ERROR; 
    
    switch(pin)
		{
				case 0: portx = GPIO_PIN1_PORT; piny = GPIO_PIN1_PIN; break;
				case 1: portx = GPIO_PIN2_PORT; piny = GPIO_PIN2_PIN; break;
				case 2: portx = GPIO_PIN3_PORT; piny = GPIO_PIN3_PIN; break;
				default:break;
		}
    
    value = (int)(HAL_GPIO_ReadPin(portx, piny));
    
    return value;    
}


static rt_err_t stm_pin_attach_irq(struct rt_device *device, rt_int32_t pin,
                                   rt_uint32_t mode, void (*hdr)(void *args), void *args)
{
    GPIO_TypeDef* portx;
		int piny, trigger_mode, i;    
    
    if(pin > PIN_MAX_VAL)
        return RT_ERROR; 
    switch(pin)
		{
				case 0: portx = GPIO_PIN1_PORT; piny = GPIO_PIN1_PIN; break;
				case 1: portx = GPIO_PIN2_PORT; piny = GPIO_PIN2_PIN; break;
				case 2: portx = GPIO_PIN3_PORT; piny = GPIO_PIN3_PIN; break;
				default:break;
		}
    
    switch (mode)
			{
			case PIN_IRQ_MODE_RISING:
					trigger_mode = GPIO_MODE_IT_RISING;
					break;
			case PIN_IRQ_MODE_FALLING:
					trigger_mode = GPIO_MODE_IT_FALLING;
					break;
			case PIN_IRQ_MODE_RISING_FALLING:
					trigger_mode = GPIO_MODE_IT_RISING_FALLING;
					break;
			case PIN_IRQ_MODE_HIGH_LEVEL:
					return -RT_ENOSYS;
					//break;
			case PIN_IRQ_MODE_LOW_LEVEL:
					return -RT_ENOSYS;
					//break;
			}
     
    /* Get inputmux_connection_t */        
    //pintsel = (pin + (0xC0U << 20));
    
    for(i = 0; i < IRQ_MAX_VAL; i++)
    {
        if(pin_irq_hdr_tab[i].pin == -1)
        {
            //pin_initx = kPINT_PinInt0 + i;
            pin_irq_hdr_tab[i].pin = pin;
            pin_irq_hdr_tab[i].mode = trigger_mode;
            pin_irq_hdr_tab[i].hdr = hdr;
            pin_irq_hdr_tab[i].args = args;
            break;
        }
    }
        
    if(i >= IRQ_MAX_VAL)
        return RT_ERROR;
		
		{
		 rt_uint32_t temp = EXTI->RTSR1;
     temp &= ~((uint32_t)piny);
			if((trigger_mode & RISING_EDGE) == RISING_EDGE)
			{
				temp |= piny;//iocurrent;
			}
			EXTI->RTSR1 = temp;

			temp = EXTI->FTSR1;
			temp &= ~((uint32_t)piny);//iocurrent;
			if((trigger_mode & FALLING_EDGE) == FALLING_EDGE)
			{
				temp |= piny;//iocurrent;
			}
			EXTI->FTSR1 = temp;
    
		}

    
    
    return RT_EOK;
}
static rt_err_t stm_pin_detach_irq(struct rt_device *device, rt_int32_t pin)
{
    int i;    
    
    if(pin > PIN_MAX_VAL)
        return RT_ERROR; 
    
    for(i = 0; i < IRQ_MAX_VAL; i++)
    {
        if(pin_irq_hdr_tab[i].pin == pin)
        {
            pin_irq_hdr_tab[i].pin = -1;
            pin_irq_hdr_tab[i].hdr = RT_NULL;
            pin_irq_hdr_tab[i].mode = 0;
            pin_irq_hdr_tab[i].args = RT_NULL;
            break;
        }
    }        
    return RT_EOK;
}
static rt_err_t stm_pin_irq_enable(struct rt_device *device, rt_base_t pin,
                                   rt_uint32_t enabled)
{  
    int irqn_type, i;
    
    if(pin > PIN_MAX_VAL)
        return RT_ERROR; 
    
    for(i = 0; i < IRQ_MAX_VAL; i++)
    {
        if(pin_irq_hdr_tab[i].pin == pin)
        {
						rt_uint16_t pinx= 0;
            switch(i)
            {
                case 0: irqn_type = GPIO_PIN1_IRQ; pinx = GPIO_PIN1_PIN; break;
                case 1: irqn_type = GPIO_PIN2_IRQ; pinx = GPIO_PIN2_PIN; break;
                case 2: irqn_type = GPIO_PIN3_IRQ; pinx = GPIO_PIN3_PIN; break;
                default:break;
            }
            if(enabled)
            {
                /* PINT_EnableCallback */
								__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);
                NVIC_ClearPendingIRQ((IRQn_Type)irqn_type);
								//__HAL_GPIO_EXTI_CLEAR_IT(pinx);
							  EXTI_D1->IMR1 |= pinx;
								HAL_NVIC_EnableIRQ((IRQn_Type)irqn_type);
            }
            else
            {
                /* PINT_DisableCallback */
								if ((get_irq_unmask((IRQn_Type)irqn_type) & ~pinx)==0){
									HAL_NVIC_DisableIRQ((IRQn_Type)irqn_type);
									__HAL_GPIO_EXTI_CLEAR_IT(pinx);
									EXTI_D1->IMR1 &= ~pinx;
									NVIC_ClearPendingIRQ((IRQn_Type)irqn_type);
								}
            }
            break;
        }
    }  
        
    if(i >= IRQ_MAX_VAL)
        return RT_ERROR;

      return RT_EOK;
}


const static struct rt_pin_ops _stm32_pin_ops =
{
    stm_pin_mode,
    stm_pin_write,
    stm_pin_read,
    stm_pin_attach_irq,
    stm_pin_detach_irq,
    stm_pin_irq_enable,    
};
static void stm32_gpio_hw_init(void){
	
	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_CLK_ENABLE();
	
	GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
	
	GPIO_InitStruct.Pin 			= GPIO_PIN1_PIN;
  HAL_GPIO_Init(GPIO_PIN1_PORT, &GPIO_InitStruct);	

	GPIO_InitStruct.Pin 			= GPIO_PIN2_PIN;
  HAL_GPIO_Init(GPIO_PIN2_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin 			= GPIO_PIN3_PIN;
  HAL_GPIO_Init(GPIO_PIN3_PORT, &GPIO_InitStruct);			
}
int stm32_hw_gpio_init(void){
	stm32_gpio_hw_init();
	return rt_device_pin_register("pin", &_stm32_pin_ops, RT_NULL);
}
INIT_BOARD_EXPORT(stm32_hw_gpio_init);
#endif
