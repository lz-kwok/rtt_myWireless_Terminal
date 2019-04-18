#include "system.h"


static rt_thread_t task1 = RT_NULL, task2 = RT_NULL, task3 = RT_NULL;

//��ӳ�䴮��1��rt_kprintf
void rt_hw_console_output(const char *str)
{
    /* empty console output */
	
	rt_enter_critical();

//	while(*str!='\0')
//	{
//		if(*str=='\n')
//		{
//			USART2->DR = (u8)  '\r'; 
//			while((USART2->SR&0X40)==0);
//		}
//		USART2->DR =*str++;
//		while((USART2->SR&0X40)==0);	
//	}

	rt_exit_critical();
}

/*******************************************************************************
* ����	    : ��ʼ�����ݿ�
*******************************************************************************/
static void InitData(void)
{
    AppDataPointer = &(App.Data);
    
    AppDataPointer->Adc.A0 = 2400;
    App.Data.Voltage = 42;
    
    ABValue = FlashRead(0);
    if(ABValue > 3)
      ABValue = 1;
    
    TurnoffIndex = FlashRead(1);
    if(TurnoffIndex > 20)
      TurnoffIndex = 1;
    
    InOutValue = FlashRead(2);
    if(InOutValue > 20)
      InOutValue = 1;
}




int main()
{
    InitData();
	//�����߳�thread_uartDMA_receive_entry
    task1 = rt_thread_create("thread_uartDMA_receive_entry", 			    //�̵߳�������thread_uartDMA_receive_entry
							thread_uartDMA_receive_entry, RT_NULL, 	        //�����thread_uartDMA_receive_entry��������RT_NULL 
							2048, 											//�̶߳�ջ��С
							1, 												//�߳����ȼ�  ����Ϊ������ȼ�
							30);											//ʱ��Ƭtick

    if (task1 != RT_NULL)                //�������߳̿��ƿ飬��������߳�
        rt_thread_startup(task1);
    else
        rt_kprintf("create thread_uartDMA_receive_entry failed!\r\n");
		
    //�����߳�thread_Menu_show_entry
    task2 = rt_thread_create("thread_Menu_show_entry", 			            //�̵߳�������thread_Menu_show_entry
                            thread_Menu_show_entry, RT_NULL,                //�����thread_Menu_show_entry��������RT_NULL 
                            2048,  									        //�̶߳�ջ��С
                            4, 											    //�߳����ȼ�
                            20);										    //ʱ��Ƭtick
	
    if (task2 != RT_NULL) 								                    // �������߳̿��ƿ飬��������߳� 
        rt_thread_startup(task2);
    else
        rt_kprintf("create thread_Menu_show_entry failed!\r\n");
		
				//�����߳�2
    task3 = rt_thread_create("thread_Key_scan_entry", 			            //�̵߳�������thread_Key_scan_entry
                            thread_Key_scan_entry, RT_NULL,                 //�����thread_Key_scan_entry��������RT_NULL 
                            512,  									        //�̶߳�ջ��С
                            3, 											    //�߳����ȼ�
                            20);										    //ʱ��Ƭtick

    if (task3 != RT_NULL) 								                    // �������߳̿��ƿ飬��������߳� 
        rt_thread_startup(task3);
    else
        rt_kprintf("create thread_Key_scan_entry failed!\r\n");

    
	return 0;
}

 



