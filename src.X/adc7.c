 /* ********************************************************************
FILE                   : adc7.c

PROGRAM DESCRIPTION    :  construction automation. mixture of cement, sand and water in the construction equiment mixture vessel and stir well.
 
    First cement, sand and water are fed in the mixture vessel. Reserviour cement sensor(RSV_CMT_SENSOR) is used to indicate level of cement in a cement reserviour. 
    Sand sensor(RSV_SAND_SENSOR) is used to indicate level of sand in a sand reserviour. Water sensor(RSV_WATER_SENSOR) is used to indicate level of water in a water reserviour.	
	
	STAGE 1:
	If reserviour cement sensor's value is above specific thresold level, then run cement flow control motor. 
	Cement quantity Sensor(VSL_CMT_SENSOR) is used to monitor quantity of cement flown in mixture vessel.
	After specific required quantity of cement is fed in mixture vessel, stop run cement flow control motor.
	If Cement quantity Sensor in mixture vessel does not reach the specific required quantity of cement in the mixture vessel 
	within a specific time duration from time of start of running the cement flow control motor, 
	the stop the cement flow control motor and sound alarm (ALARM_CMT_QTY)by flashing led on for cement indicator as there may be problem of cement leavel or 
	cement related sensors is not proper functioning. If reserviour cement sensor's value is less than specific thresold level, 
	then alarm sound(ALARM_CMT_LVL) by flashing led on. 
	
    STAGE 2:
    If required quantity of cement is fed in the mixture vessel, and if reserviour sand sensor's value is above specific thresold level,
	then run sand flow control motor. Sand quantity Sensor(VSL_SAND_SENSOR) is used to monitor quantity of sand flown in mixture vessel.
	After specific required quantity of sand is fed in mixture vessel, stop run sand flow control motor.
	If sand quantity Sensor in mixture vessel does not reach the specific required quantity of sand in the mixture vessel 
	within a specific time duration from time of start of running the sand flow control motor, 
	the stop the sand flow control motor and sound alarm (ALARM_SAND_QTY)by flashing led on for sand indicator as there may be problem of sand level or 
	sand related sensors is not proper functioning. If reserviour sand sensor's value is less than specific thresold level, 
	then alarm sound(ALARM_SAND_LVL) by flashing led on. 
	
   STAGE 3:
    If required quantity of sand and cement are fed in the mixture vessel, and if reserviour water sensor's value is above specific thresold level,
	then run water pump. water quantity Sensor(VSL_WATER_SENSOR) is used to monitor quantity of water flown in mixture vessel.
	After specific required quantity of water is fed in mixture vessel, stop run water flow control motor.
	If water quantity Sensor in mixture vessel does not reach the specific required quantity of water in the mixture vessel 
	within a specific time duration from time of start of running the water flow control motor, 
	the stop the water flow control motor and sound alarm (ALARM_WATER_QTY)by flashing led on for water indicator as there may be problem of water leavel or 
	sand related sensors is not proper functioning. If reserviour water sensor's value is less than specific thresold level, 
	then alarm sound(ALARM_WATER_LVL) by flashing led on. 
	
	STAGE 4:
	After required quantity of sand, cement and water are fed in the mixture vessel. Start run mixture vessel rotoring control motor 
	(mixture of cement, sand and water) in foward direction for 3 minutes and then run mixture rotoring control motor in reverse dirction for 3 minutes. 
	Repeat this process of forward and then reverse run of mixture vessel rotoring control motor for another 2 times. 
	Then stop mixture rotoring control motor and flash led on (MIXED_OK)to indicate the mixture of cement, sand and water are stired well and complete.
	
	After a RESTART_SW is pressed to indicate that stirred well mixture of cement, sand and water are taken out of mixture vessel 
	and ready to receive fresh intake of cement, sand and water in the mixture vessel. Repeat the process.
	

AUTHOR                : K.M.Arun Kumar alias Arunkumar Murugeswaran
	 
KNOWN BUGS            : 

NOTE                  :  Display control and display datas in LCD are not fully implemented
                                  
                       
CHANGE LOGS           : 

*****************************************************************************/    
	
#include <xc.h>
#define RS_PIN                                 RD0
#define RW_PIN                                 RD1
#define EN_PIN                                 RD2
#define LCD_PORT                              PORTC

#define RELAY_MIXVSL_REV_RUN_PIN                 RE0 
#define RELAY_MIXVSL_FWD_RUN_PIN                 RE1
#define RELAY_CMT_FWD_RUN_PIN                    RD4
#define RELAY_SAND_FWD_RUN_PIN                   RD5
#define RELAY_WATER_FWD_RUN_PIN                  RD6

#define MIX_COMPLETED_PIN                        RB6
#define RESTART_SW                               RE2

//RSV - Reseveriour, VSL - MIXTURE VESSEL, CMT- Cement, ALM - Alarm, QTY - Quantity, CNT - Count
#define ALM_CMT_RSV_PIN                         RB7
#define ALM_CMT_VSL_PIN                         RB1 
#define ALM_SAND_RSV_PIN                        RB2
#define ALM_SAND_VSL_PIN                        RB3
#define ALM_WATER_RSV_PIN                       RB4
#define ALM_WATER_VSL_PIN                       RB5 
       
#define RELAY_ON                                (1)
#define RELAY_OFF                               (0)
#define SWITCH_PRESSED_ON                       (1)
#define SWITCH_NOT_PRESSED                      (0)
#define LED_ON                                  (1)
#define LED_OFF                                 (0) 
#define STATE_YES                               ('y')
#define STATE_NO                                ('n')
 /* should status be displayed on or off for error, warning and current time left */
#define STATUS_DISP_ON                            (1)  
#define STATUS_DISP_OFF                           (0) 

#define DISP_FLAG_NUM_DIGIT1                   (1)
#define DISP_FLAG_NUM_DIGIT2                   (2)
#define DISP_FLAG_NUM_DIGIT3                   (3)
#define DISP_FLAG_NUM_DIGIT4                   (4)
#define DISP_FLAG_NUM_DIGIT5                   (5)
#define DISP_FLAG_HEX_DIGIT1                   (6)
#define DISP_FLAG_HEX_DIGIT2                   (7)
#define DISP_FLAG_HEX_DIGIT3                   (8)
#define DISP_FLAG_HEX_DIGIT4                   (9) 

/* for 20 * 4 LCD disp */                             
#define BEGIN_LOC_LINE1                      (0X80)
#define BEGIN_LOC_LINE2                      (0xC0)
#define BEGIN_LOC_LINE3                      (0x94) 
#define BEGIN_LOC_LINE4                      (0xD4)
#define END_LOC_LINE1                        (0x93)
#define END_LOC_LINE2                        (0xD3)
#define END_LOC_LINE3                        (0xA7) 
#define END_LOC_LINE4                        (0xE7)

/* num cols = num of chars in a line */
#define MAX_COUNT_DELAY_TIME_LCDPULSE     (100UL)
#define MAX_AVAIL_NUM_COLS                    (20U)
#define CONFIGURE_MAX_NUM_LINES               (4U)
#define MAX_AVAIL_NUM_LINES                   (4U) 
#define MAX_AVAIL_NUM_CHARS_IN_LCD        (MAX_AVAIL_NUM_COLS * MAX_AVAIL_NUM_LINES) 
#define CONFIGURE_MAX_NUM_COLS             (MAX_AVAIL_NUM_COLS)
#define CONFIGURE_MAX_NUM_CHARS_IN_LCD    (CONFIGURE_MAX_NUM_LINES * CONFIGURE_MAX_NUM_COLS ) 
#define MAX_NUM_CHARS_INPUT_DATA          (MAX_AVAIL_NUM_COLS)  

#define INVALID_DATA               (0U)
#define ALL_LINES                  (0U)
#define NUM_LINE1                  (1U)
#define NUM_LINE2                  (2U)
#define NUM_LINE3                  (3U)
#define NUM_LINE4                  (4U)
#define NUM_COL1                   (1U)

#define RSV_CHS_LINE_NUM             (NUM_LINE1)
#define VSL_CHS_LINE_NUM             (NUM_LINE2)
#define TIME_STATUS_LINE_NUM          (NUM_LINE3)
#define MOTOR_STATUS_LINE_NUM         (NUM_LINE3) 
#define LED_STATUS_LINE_NUM           (NUM_LINE4)
/* START column*/
#define RSV_CMT_PERCENT_INT_COL_NUM       (NUM_COL1 + 2)
#define RSV_CMT_PERCENT_DECPT_COL_NUM     (RSV_CMT_PERCENT_INT_COL_NUM + 3)
#define RSV_CMT_PERCENT_FRAC_COL_NUM      (RSV_CMT_PERCENT_DECPT_COL_NUM + 1) 
#define RSV_SAND_PERCENT_INT_COL_NUM       (RSV_CMT_PERCENT_FRAC_COL_NUM + 2)
#define RSV_SAND_PERCENT_DECPT_COL_NUM     (RSV_SAND_PERCENT_INT_COL_NUM + 3)
#define RSV_SAND_PERCENT_FRAC_COL_NUM      (RSV_SAND_PERCENT_DECPT_COL_NUM + 1) 
#define RSV_WATER_PERCENT_INT_COL_NUM       (RSV_SAND_PERCENT_FRAC_COL_NUM + 2)
#define RSV_WATER_PERCENT_DECPT_COL_NUM     (RSV_WATER_PERCENT_INT_COL_NUM + 3)
#define RSV_WATER_PERCENT_FRAC_COL_NUM      (RSV_WATER_PERCENT_DECPT_COL_NUM + 1)  
#define RSV_CHS_PERCENT_MSG_COL_NUM   (RSV_WATER_PERCENT_FRAC_COL_NUM + 1)  

#define VSL_CMT_INT_COL_NUM           (NUM_COL1 + 2)
#define VSL_SAND_INT_COL_NUM           (VSL_CMT_INT_COL_NUM + 5)
#define VSL_WATER_INT_COL_NUM           (VSL_SAND_INT_COL_NUM + 5)
#define VSL_CHS_VOLUME_MSG_COL_NUM       (VSL_WATER_INT_COL_NUM + 4)

#define TIME_STATUS_COL_NUM           (NUM_COL1 + 2) 
#define COUNT_MSG_COL_NUM             (TIME_STATUS_COL_NUM + 3)
#define COUNT_STATUS_COL_NUM          (COUNT_MSG_COL_NUM + 3)
#define MOTOR_MSG_COL_NUM             (COUNT_STATUS_COL_NUM + 2)
#define MOTOR_STATUS_COL_NUM          (MOTOR_MSG_COL_NUM + 3)

#define TMR1_OFF_STATE             (0U)
#define TMR1_MODE_VSL_CMT          (1U)
#define TMR1_MODE_VSL_SAND         (2U)
#define TMR1_MODE_VSL_WATER        (3U)
#define TMR1_MODE_VSL_RUN          (4U)


#define MIX_FSM_INITIAL             (0U)
#define MIX_FSM_VSL_CMT            (1U)
#define MIX_FSM_VSL_SAND           (2U)
#define MIX_FSM_VSL_WATER          (3U)
#define MIX_FSM_VSL_RUN            (4U) 
#define MIX_FSM_STIR_COMPLETE      (5U)
#define MIX_FSM_STOP               (6U)

#define VSL_RUN_FSM_FWD             (1U)
#define VSL_RUN_FSM_REV             (2U)
#define VSL_NO_RUN_FSM              (3U)

/* disp_status_time_or_error[] index ie disp_status_time_or_error[TIME_STATUS_DISP_INDEX] = STATUS_DISP_ON, \
   then time status is displayed */
#define ERROR_STATUS_DISP_INDEX             (0U)
#define TIME_STATUS_DISP_INDEX              (1U)
#define MOTOR_STATUS_DISP_INDEX             (2U)
#define LED_STATUS_DISP_INDEX               (3U)

#define ANY_DATA_DISP                 (1U)
#define BLANK_LINE_DISP               (2U)
#define TIME_DISP                     (3U) 
#define MOTOR_DISP                    (4U)  
#define VSL_RUN_DISP                  (6U)
#define LED_DISP                      (7U)  
#define ERROR_DISP                    (8U)
#define WARNING_DISP                  (9U)
#define FATAL_DISP                    (10U) 

#define RSV_CMT_SENSOR_CH                (0U)
#define RSV_SAND_SENSOR_CH               (1U)
#define RSV_WATER_SENSOR_CH              (2U)
#define VSL_CMT_SENSOR_CH                (3U)
#define VSL_SAND_SENSOR_CH               (12U)
#define VSL_WATER_SENSOR_CH              (4U)
          

#define _XTAL_FREQ               (4000000UL)
// TIMER1 expires every 50ms and  
#define TIMER1_TICK_MSEC        (50UL) 
#define TIME_UNIT_SEC_TO_MSEC    (1000UL)
#define OSC_PER_INST                (4)
#define INC1          (unsigned long)((unsigned long)(_XTAL_FREQ * TIMER1_TICK_MSEC) / (unsigned long)(OSC_PER_INST * TIME_UNIT_SEC_TO_MSEC)) 

#define TIME_UPDATE                  (500UL) 
#define CUR_STAGE_TIME_FACTOR_PER_SEC (1000UL/TIME_UPDATE)
#define UPDATE_TIME1_CUR_STAGE        (TIME_UPDATE/TIMER1_TICK_MSEC)

#define MAX_RSV_CMT_QTY_IN_VAL       (1023UL)
#define MIN_RSV_CMT_QTY_IN_VAL       (0U)
#define FULL_SCALE_RSV_CMT_QTY       ((MAX_RSV_CMT_QTY_IN_VAL) - (MIN_RSV_CMT_QTY_IN_VAL))
 
#define MAX_RSV_SAND_QTY_IN_VAL       (1023UL)
#define MIN_RSV_SAND_QTY_IN_VAL       (0U)
#define FULL_SCALE_RSV_SAND_QTY      ((MAX_RSV_SAND_QTY_IN_VAL) - (MIN_RSV_SAND_QTY_IN_VAL))

#define MAX_RSV_WATER_QTY_IN_VAL       (1023UL)
#define MIN_RSV_WATER_QTY_IN_VAL       (0U)
#define FULL_SCALE_RSV_WATER_QTY       ((MAX_RSV_WATER_QTY_IN_VAL) - (MIN_RSV_WATER_QTY_IN_VAL))

#define MAX_VSL_CMT_QTY_IN_VAL           (512UL)
#define MIN_VSL_CMT_QTY_IN_VAL           (0U)
#define FULL_SCALE_VSL_CMT_QTY         ((MAX_VSL_CMT_QTY_IN_VAL) - (MIN_VSL_CMT_QTY_IN_VAL))

#define MAX_VSL_SAND_QTY_IN_VAL           (512UL)
#define MIN_VSL_SAND_QTY_IN_VAL           (0U)
#define FULL_SCALE_VSL_SAND_QTY           ((MAX_VSL_SAND_QTY_IN_VAL) - (MIN_VSL_SAND_QTY_IN_VAL))

#define MAX_VSL_WATER_QTY_IN_VAL           (512UL)
#define MIN_VSL_WATER_QTY_IN_VAL           (0U)
#define FULL_SCALE_VSL_WATER_QTY           ((MAX_VSL_WATER_QTY_IN_VAL) - (MIN_VSL_WATER_QTY_IN_VAL))

#define ADC_ACQUIST_TIME_IN_CNT      (1000UL)
#define ADC_CH_SELECT_TIME_IN_CNT    (1000UL)

#define MIN_RSV_CMT_QTY_IN_PERCENT      (25U)
#define MIN_RSV_SAND_QTY_IN_PERCENT     (25U)
#define MIN_RSV_WATER_QTY_IN_PERCENT    (25U)
#define REQ_VSL_CMT_QTY_IN_VAL          (100U) 
#define REQ_VSL_SAND_QTY_IN_VAL         (100U)
#define REQ_VSL_WATER_QTY_IN_VAL        (100U)

#define MAX_TIME_VSL_CMT_FILL_IN_SEC        (15U)
#define MAX_TIME_VSL_SAND_FILL_IN_SEC       (15U)          
#define MAX_TIME_VSL_WATER_FILL_IN_SEC      (15U) 

#define REQ_FWD_RUN_STIR_IN_SEC            (10U)
#define REQ_REV_RUN_STIR_IN_SEC            (12U)
#define REQ_REPEAT_FWD_REV_PROCESS_IN_CNT  (3U)


void Delay_Time_ByCount(unsigned int Delay_Time_Count_count);
void LCD_Init();
void LCD_Pulse ();
void Write_LCD_Command (const unsigned int Write_LCD_Command);
void Write_LCD_Data(const char lcd_disp_ch);
void Data_Str_Disp_LCD(const char *lcd_disp_str);
void Data_Num_Disp_LCD(const unsigned int lcd_datanum_disp_format, const unsigned long lcd_disp_data_int);
void LCD_Const_Disp();
void Goto_XY_LCD_Disp(const unsigned int start_line_num, const unsigned int start_col_num);

void Run_Timer1(const unsigned int set_timer1_mode );
void Prescale_Timer1();
void Stop_Timer1();
void Cur_Stage_Time_Run_Proc();
void Cur_Stage_Time_Proc();

void Init_ADC_Channel(const unsigned int );
unsigned long Read_ADC_Channel(unsigned int adc_channel);
void Encoded_To_Actual_Analog_Val_Calc(const unsigned long adc_value, const unsigned long full_scale_input_analog, const unsigned min_input_analog, \
   unsigned long *const analog_val_in_digital_int, unsigned long *const analog_val_in_digital_frac );
void Encoded_To_Percent_Calc(const unsigned long adc_value, unsigned int *const percent_int, unsigned int *const percent_frac );


void Reset_Process();
void Mix_Fsm_Proc();

/* currently displayed data in each line starts from 1 ie use array index as line num for us. index 0 can be used as all lines */
unsigned int cur_line_disp_data[] = {ANY_DATA_DISP, ANY_DATA_DISP, ANY_DATA_DISP, BLANK_LINE_DISP, BLANK_LINE_DISP};
unsigned int prescale_timer1 = 0x01, prescale_shift_timer1= 0, timer1_mode = TMR1_OFF_STATE;
unsigned long int num_calls_timer1 = 0, timer1_init = 0;
unsigned int count_update_cur_stage_per_sec = 0;

unsigned int disp_status_time_or_error[] = {STATUS_DISP_ON, STATUS_DISP_ON, STATUS_DISP_ON,STATUS_DISP_OFF};

unsigned int cur_disp_lcd_loc = BEGIN_LOC_LINE1;
unsigned int mix_fsm_state, vsl_run_fsm_state;
unsigned int cur_stage_time_left, count_left_vsl_fwd_rev_process; 
char restart_sw_enable_flag = STATE_YES, restart_pressed_flag, cur_stage_time_enable_flag = STATE_NO, cur_stage_time_expiry_flag = STATE_NO, mix_process_enable_flag = STATE_YES; 
void main()
{
	unsigned long analog_val_in_digital_int_ch0, analog_val_in_digital_frac_ch0, analog_val_in_digital_int_ch1, analog_val_in_digital_frac_ch1;
	unsigned int percent_int_ch0, percent_frac_ch0, percent_int_ch1, percent_frac_ch1;	
	const char motor_off_msg_disp[] = "OFF", motor_fwd_run_msg_disp[] = "FWD",motor_rev_run_msg_disp[] = "REV"; 
	TRISA = 0x2F;
	PORTA = 0x00; 
	TRISB = 0x00;
	PORTB = 0x00;
	TRISC = 0x00;
	PORTC = 0x00;
	TRISD = 0x00;
	PORTD = 0x00;
	TRISE = 0X04;
	PORTE = 0x00;
	ANSEL = 0X0F;
	ANSELH = 0x10;
	LCD_Init(); 	
	LCD_Const_Disp();
    //Reset_Process();	
	for(;;)
    {    
        if(restart_sw_enable_flag == STATE_YES && RESTART_SW == SWITCH_PRESSED_ON)
		{
			while(RESTART_SW == SWITCH_PRESSED_ON);
			Reset_Process();
			restart_pressed_flag = STATE_YES;
			restart_sw_enable_flag = STATE_NO;			  
        }
		if(mix_process_enable_flag == STATE_YES)
           Mix_Fsm_Proc();   
       if((timer1_mode != TMR1_OFF_STATE ) && cur_stage_time_expiry_flag == STATE_NO && cur_stage_time_enable_flag == STATE_YES)   
		  Cur_Stage_Time_Run_Proc();
	}		
}
void Reset_Process()
{
	Stop_Timer1();
	ALM_CMT_RSV_PIN   = LED_OFF;                      
    ALM_CMT_VSL_PIN   = LED_OFF;                      
    ALM_SAND_RSV_PIN  = LED_OFF;                    
    ALM_SAND_VSL_PIN   = LED_OFF;                     
    ALM_WATER_RSV_PIN  = LED_OFF;                     
    ALM_WATER_VSL_PIN = LED_OFF;
	MIX_COMPLETED_PIN = LED_OFF;
	restart_sw_enable_flag = STATE_YES;
    cur_stage_time_expiry_flag = STATE_NO;
    cur_stage_time_enable_flag = STATE_NO;	
	mix_fsm_state = MIX_FSM_INITIAL; 
    Write_LCD_Command(0x01);
	LCD_Const_Disp();
   	cur_line_disp_data[TIME_STATUS_LINE_NUM] = BLANK_LINE_DISP;
}
/*------------------------------------------------------------*
FUNCTION NAME  : 

DESCRIPTION    :
								
INPUT          : none

OUTPUT         : 

NOTE           : 

Func ID        : 7   
-*------------------------------------------------------------*/
void Mix_Fsm_Proc()
{
	unsigned long analog_val_in_digital_int_vslcmt_ch, analog_val_in_digital_frac_vslcmt_ch, analog_val_in_digital_int_vslsand_ch, \
       analog_val_in_digital_frac_vslsand_ch,analog_val_in_digital_int_vslwater_ch, analog_val_in_digital_frac_vslwater_ch ;
	unsigned int percent_int_rsvcmt_ch, percent_frac_rsvcmt_ch, percent_int_rsvsand_ch, percent_frac_rsvsand_ch, percent_int_rsvwater_ch, percent_frac_rsvwater_ch ;	
    unsigned long adc_value_rsvcmt_ch, adc_value_rsvsand_ch, adc_value_rsvwater_ch, adc_value_vslcmt_ch, adc_value_vslsand_ch, adc_value_vslwater_ch; 
    unsigned int adc_channel;
	switch(mix_fsm_state)
	{
		case MIX_FSM_INITIAL:
		  if(restart_pressed_flag == STATE_YES)
		  {	  
		    cur_stage_time_left = MAX_TIME_VSL_CMT_FILL_IN_SEC;
			cur_stage_time_enable_flag = STATE_YES;
	        Run_Timer1(TMR1_MODE_VSL_CMT);
			mix_fsm_state = MIX_FSM_VSL_CMT;
			restart_pressed_flag = STATE_NO;
		  }	
		break;
		case MIX_FSM_VSL_CMT:
		  adc_channel = RSV_CMT_SENSOR_CH;
		  adc_value_rsvcmt_ch =  Read_ADC_Channel(RSV_CMT_SENSOR_CH);
		  Encoded_To_Percent_Calc(adc_value_rsvcmt_ch,&percent_int_rsvcmt_ch,&percent_frac_rsvcmt_ch);
		  Goto_XY_LCD_Disp(RSV_CHS_LINE_NUM, RSV_CMT_PERCENT_INT_COL_NUM);
		  Data_Num_Disp_LCD(DISP_FLAG_NUM_DIGIT3,percent_int_rsvcmt_ch );
		  Goto_XY_LCD_Disp(RSV_CHS_LINE_NUM, RSV_CMT_PERCENT_FRAC_COL_NUM);
		  Data_Num_Disp_LCD(DISP_FLAG_NUM_DIGIT1, percent_frac_rsvcmt_ch);		  
		  if(percent_int_rsvcmt_ch < MIN_RSV_CMT_QTY_IN_PERCENT)
		  {
			  cur_stage_time_enable_flag = STATE_NO;
              ALM_CMT_RSV_PIN = LED_ON;
              restart_sw_enable_flag = STATE_YES;
			  Stop_Timer1();
			  mix_fsm_state = MIX_FSM_STOP;		     	
              break;			 
          }
		  if(cur_stage_time_expiry_flag == STATE_YES)
		  {
			  Stop_Timer1();
			  cur_stage_time_enable_flag = STATE_NO;
		      ALM_CMT_VSL_PIN = LED_ON;
			  RELAY_CMT_FWD_RUN_PIN = RELAY_OFF;
			  mix_fsm_state = MIX_FSM_STOP;		      	  
			  break;
		  }
		  RELAY_CMT_FWD_RUN_PIN = RELAY_ON;
          Delay_Time_ByCount(ADC_CH_SELECT_TIME_IN_CNT);	 		  
		  adc_channel = VSL_CMT_SENSOR_CH;
	      adc_value_vslcmt_ch =  Read_ADC_Channel(VSL_CMT_SENSOR_CH);
		  Encoded_To_Actual_Analog_Val_Calc(adc_value_vslcmt_ch, FULL_SCALE_VSL_CMT_QTY, MIN_VSL_CMT_QTY_IN_VAL,\
    		   &analog_val_in_digital_int_vslcmt_ch,&analog_val_in_digital_frac_vslcmt_ch );
		  Goto_XY_LCD_Disp(VSL_CHS_LINE_NUM, VSL_CMT_INT_COL_NUM);
          Data_Num_Disp_LCD(DISP_FLAG_NUM_DIGIT4,analog_val_in_digital_int_vslcmt_ch );		 
          if(analog_val_in_digital_int_vslcmt_ch >= REQ_VSL_CMT_QTY_IN_VAL)
          {
			  cur_stage_time_enable_flag = STATE_NO;
			  RELAY_CMT_FWD_RUN_PIN = RELAY_OFF;
			  Stop_Timer1();
			  mix_fsm_state = MIX_FSM_VSL_SAND; 
              cur_stage_time_enable_flag = STATE_YES;			  
			  cur_stage_time_left = MAX_TIME_VSL_SAND_FILL_IN_SEC; 
			  Run_Timer1(TMR1_MODE_VSL_SAND);			  
			  //SHOULD_REMOVE
			  Goto_XY_LCD_Disp(4,4);
			  Data_Str_Disp_LCD("CM"); 
          }		  			  
		break;
        case MIX_FSM_VSL_SAND:
		  Delay_Time_ByCount(ADC_CH_SELECT_TIME_IN_CNT);
		  adc_channel = RSV_SAND_SENSOR_CH;
		  adc_value_rsvsand_ch =  Read_ADC_Channel(RSV_SAND_SENSOR_CH);
		  Encoded_To_Percent_Calc(adc_value_rsvsand_ch,&percent_int_rsvsand_ch,&percent_frac_rsvsand_ch );
		  Goto_XY_LCD_Disp(RSV_CHS_LINE_NUM, RSV_SAND_PERCENT_INT_COL_NUM);
		  Data_Num_Disp_LCD(DISP_FLAG_NUM_DIGIT3,percent_int_rsvsand_ch);
		  Goto_XY_LCD_Disp(RSV_CHS_LINE_NUM, RSV_SAND_PERCENT_FRAC_COL_NUM);
		  Data_Num_Disp_LCD(DISP_FLAG_NUM_DIGIT1,percent_frac_rsvsand_ch );	
		  if(percent_int_rsvsand_ch < MIN_RSV_SAND_QTY_IN_PERCENT)
		  {
			  cur_stage_time_enable_flag = STATE_NO;
              ALM_SAND_RSV_PIN = LED_ON;              
			  Stop_Timer1();
			  mix_fsm_state = MIX_FSM_STOP;		     	
              break;			 
          }
		  if(cur_stage_time_expiry_flag == STATE_YES)
		  {
			  cur_stage_time_enable_flag = STATE_NO;
			  Stop_Timer1();
			  ALM_SAND_VSL_PIN = LED_ON;
			  RELAY_SAND_FWD_RUN_PIN = RELAY_OFF;
              mix_fsm_state = MIX_FSM_STOP;			  
			  break;
		  }	
		  RELAY_SAND_FWD_RUN_PIN = RELAY_ON;
          Delay_Time_ByCount(ADC_CH_SELECT_TIME_IN_CNT);		  
		  adc_channel = VSL_SAND_SENSOR_CH ;
	      adc_value_vslsand_ch =  Read_ADC_Channel(VSL_SAND_SENSOR_CH);
		  Encoded_To_Actual_Analog_Val_Calc(adc_value_vslsand_ch, FULL_SCALE_VSL_SAND_QTY, MIN_VSL_SAND_QTY_IN_VAL,\
    		   &analog_val_in_digital_int_vslsand_ch,&analog_val_in_digital_frac_vslsand_ch );
		  Goto_XY_LCD_Disp(VSL_CHS_LINE_NUM, VSL_SAND_INT_COL_NUM);
          Data_Num_Disp_LCD(DISP_FLAG_NUM_DIGIT4,analog_val_in_digital_int_vslsand_ch );   
          if(analog_val_in_digital_int_vslsand_ch >= REQ_VSL_SAND_QTY_IN_VAL)
          {
			  cur_stage_time_enable_flag = STATE_NO; 
			  RELAY_SAND_FWD_RUN_PIN = RELAY_OFF;
			  Stop_Timer1();
			  mix_fsm_state = MIX_FSM_VSL_WATER;			  
			  cur_stage_time_enable_flag = STATE_YES;   
			  cur_stage_time_left = MAX_TIME_VSL_WATER_FILL_IN_SEC; 
			  Run_Timer1(TMR1_MODE_VSL_WATER);
			  
			  //SHOULD_REMOVE
			  Goto_XY_LCD_Disp(4,7);
			  Data_Str_Disp_LCD("SA"); 
          }		  			  
		break;
        case MIX_FSM_VSL_WATER:
		  Delay_Time_ByCount(ADC_CH_SELECT_TIME_IN_CNT);
		  adc_channel = RSV_WATER_SENSOR_CH;
		  adc_value_rsvwater_ch =  Read_ADC_Channel(RSV_WATER_SENSOR_CH);
		  Encoded_To_Percent_Calc(adc_value_rsvwater_ch,&percent_int_rsvwater_ch,&percent_frac_rsvwater_ch );
		  Goto_XY_LCD_Disp(RSV_CHS_LINE_NUM, RSV_WATER_PERCENT_INT_COL_NUM);
		  Data_Num_Disp_LCD(DISP_FLAG_NUM_DIGIT3,percent_int_rsvwater_ch );
		  Goto_XY_LCD_Disp(RSV_CHS_LINE_NUM, RSV_WATER_PERCENT_FRAC_COL_NUM);
		  Data_Num_Disp_LCD(DISP_FLAG_NUM_DIGIT1, percent_frac_rsvwater_ch);
		  if(percent_int_rsvwater_ch < MIN_RSV_WATER_QTY_IN_PERCENT)
		  {
			  ALM_WATER_RSV_PIN = LED_ON;
              cur_stage_time_enable_flag = STATE_NO;
			  Stop_Timer1();
			  mix_fsm_state = MIX_FSM_STOP;		      	
             break;			 
          }	
		  if(cur_stage_time_expiry_flag == STATE_YES)
		  {
			 cur_stage_time_enable_flag = STATE_NO; 
			 Stop_Timer1();
			 ALM_WATER_VSL_PIN = LED_ON;
			 RELAY_WATER_FWD_RUN_PIN = RELAY_OFF;
			 mix_fsm_state = MIX_FSM_STOP;			 
             break;			 
		  }	  
		  RELAY_WATER_FWD_RUN_PIN = RELAY_ON;	
          Delay_Time_ByCount(ADC_CH_SELECT_TIME_IN_CNT);		  
		  adc_channel = VSL_WATER_SENSOR_CH ;
	      adc_value_vslwater_ch =  Read_ADC_Channel(VSL_WATER_SENSOR_CH);
		  Encoded_To_Actual_Analog_Val_Calc(adc_value_vslwater_ch, FULL_SCALE_VSL_WATER_QTY, MIN_VSL_WATER_QTY_IN_VAL,\
    		   &analog_val_in_digital_int_vslwater_ch,&analog_val_in_digital_frac_vslwater_ch );
		  Goto_XY_LCD_Disp(VSL_CHS_LINE_NUM, VSL_WATER_INT_COL_NUM);
          Data_Num_Disp_LCD(DISP_FLAG_NUM_DIGIT4, analog_val_in_digital_int_vslwater_ch);   	   
          if( analog_val_in_digital_int_vslwater_ch >= REQ_VSL_CMT_QTY_IN_VAL)
          {
			  RELAY_WATER_FWD_RUN_PIN = RELAY_OFF;	
			  cur_stage_time_enable_flag = STATE_NO;		  
			  Stop_Timer1();
			  mix_fsm_state = MIX_FSM_VSL_RUN;
			  vsl_run_fsm_state = VSL_RUN_FSM_FWD;
			  RELAY_MIXVSL_REV_RUN_PIN = LED_OFF;             
              RELAY_MIXVSL_FWD_RUN_PIN = LED_ON;
			  cur_stage_time_left = REQ_FWD_RUN_STIR_IN_SEC; 
			  count_left_vsl_fwd_rev_process = REQ_REPEAT_FWD_REV_PROCESS_IN_CNT;
			  cur_stage_time_enable_flag = STATE_YES;
			  Run_Timer1(TMR1_MODE_VSL_RUN);			 
			  
			  //SHOULD_REMOVE
			  Goto_XY_LCD_Disp(4,10);
			  Data_Str_Disp_LCD("WA");
          }
        break;
		case MIX_FSM_VSL_RUN:
		   if(cur_stage_time_expiry_flag == STATE_YES)	
           {
			    switch(vsl_run_fsm_state)
				{
				     case VSL_RUN_FSM_FWD:
                       vsl_run_fsm_state  = VSL_RUN_FSM_REV;	
				       cur_stage_time_left = REQ_REV_RUN_STIR_IN_SEC;
                       RELAY_MIXVSL_REV_RUN_PIN = RELAY_ON;             
                       RELAY_MIXVSL_FWD_RUN_PIN = RELAY_OFF;
					   cur_stage_time_expiry_flag = STATE_NO;
				       break;				
				     case VSL_RUN_FSM_REV:                       				 
                       if(--count_left_vsl_fwd_rev_process)  
				       {
					       vsl_run_fsm_state = VSL_RUN_FSM_FWD; 				  
                           cur_stage_time_left = REQ_FWD_RUN_STIR_IN_SEC;
						   cur_stage_time_expiry_flag = STATE_NO;
                           RELAY_MIXVSL_REV_RUN_PIN = RELAY_OFF;             
                           RELAY_MIXVSL_FWD_RUN_PIN = RELAY_ON;	
                           break;					  
                       }
					   Stop_Timer1();
					   cur_stage_time_enable_flag = STATE_NO;
					   RELAY_MIXVSL_REV_RUN_PIN = RELAY_OFF;             
                       RELAY_MIXVSL_FWD_RUN_PIN = RELAY_OFF;
					   MIX_COMPLETED_PIN = LED_ON;					   					 
					   mix_fsm_state = MIX_FSM_STIR_COMPLETE;
					   vsl_run_fsm_state = VSL_NO_RUN_FSM;
					   restart_sw_enable_flag = STATE_YES;					   
					   break; 
				}			   
		   }			   
		break;
        case MIX_FSM_STIR_COMPLETE:
		  mix_fsm_state = MIX_FSM_INITIAL;
		  restart_sw_enable_flag = STATE_YES;
		  cur_line_disp_data[ALL_LINES] = ANY_DATA_DISP;
		  
		  //SHOULD_REMOVE
		  Goto_XY_LCD_Disp(4,19);
		  Data_Str_Disp_LCD("OK");
						  
        break;		
        case MIX_FSM_STOP:
		/* some issue has occured before stirring the materials. in our case just for simulation just restart the mix process */
		   mix_fsm_state = MIX_FSM_INITIAL;
		   restart_sw_enable_flag = STATE_YES;
		   cur_line_disp_data[ALL_LINES] = ANY_DATA_DISP;
        break; 		
	}
}
/*------------------------------------------------------------*
FUNCTION NAME  : 

DESCRIPTION    :
								
INPUT          : none

OUTPUT         : 

NOTE           : 

Func ID        : 7   
-*------------------------------------------------------------*/
void Cur_Stage_Time_Run_Proc()
{	
    while(TMR1IF == 0); 
	
	TMR1IF = 0;
	timer1_init = (65536) - (INC1/prescale_timer1); 
    TMR1H = timer1_init / 256;
    TMR1L = timer1_init % 256; 
		
     if(++num_calls_timer1 >= UPDATE_TIME1_CUR_STAGE)
     {
	     Cur_Stage_Time_Proc();                			 
         num_calls_timer1 = 0;        
     }	 
	
}
/*------------------------------------------------------------*
FUNCTION NAME  : 

DESCRIPTION    :
								
INPUT          : none

OUTPUT         : 

NOTE           : 

Func ID        : 6  
-*------------------------------------------------------------*/
void Cur_Stage_Time_Proc()
{
	
	const char time_msg_disp[] = "T:", count_msg_disp[] = " C:", motor_msg_disp[] = " M: ";
	
	
   if(++count_update_cur_stage_per_sec % CUR_STAGE_TIME_FACTOR_PER_SEC == 0 )
   { 
        count_update_cur_stage_per_sec = 0;
       --cur_stage_time_left;    
	
       /* data is constant for a while, so if data is const,at lcd const data is displayed only once until const data at that line has changed */
	  if(disp_status_time_or_error[TIME_STATUS_DISP_INDEX] == STATUS_DISP_ON && \
	    cur_line_disp_data[TIME_STATUS_LINE_NUM] == BLANK_LINE_DISP)  
       {
		   Goto_XY_LCD_Disp(TIME_STATUS_LINE_NUM, NUM_COL1);
	       Data_Str_Disp_LCD(time_msg_disp);
		   if( mix_fsm_state == MIX_FSM_VSL_RUN)
		   {   
	          Goto_XY_LCD_Disp(TIME_STATUS_LINE_NUM, COUNT_MSG_COL_NUM);
	          Data_Str_Disp_LCD(count_msg_disp);
		   }  
           Goto_XY_LCD_Disp(MOTOR_STATUS_LINE_NUM, MOTOR_MSG_COL_NUM);
	       Data_Str_Disp_LCD(motor_msg_disp); 
           cur_line_disp_data[MOTOR_STATUS_LINE_NUM] = MOTOR_DISP;    		   
		   cur_line_disp_data[TIME_STATUS_LINE_NUM] = TIME_DISP;		        		   
      }
	   /* disp variable data every time at loc, if line has time data */
	  if( disp_status_time_or_error[TIME_STATUS_DISP_INDEX] == STATUS_DISP_ON && \
	     cur_line_disp_data[TIME_STATUS_LINE_NUM] == TIME_DISP)   
	   {
		   Goto_XY_LCD_Disp(TIME_STATUS_LINE_NUM, TIME_STATUS_COL_NUM ); 
           Data_Num_Disp_LCD(DISP_FLAG_NUM_DIGIT3, cur_stage_time_left); 
           if( mix_fsm_state == MIX_FSM_VSL_RUN)
		   {  
              Goto_XY_LCD_Disp(TIME_STATUS_LINE_NUM, COUNT_STATUS_COL_NUM ); 
              Data_Num_Disp_LCD(DISP_FLAG_NUM_DIGIT2, count_left_vsl_fwd_rev_process); 
		   }			  
       }    
       if(cur_stage_time_left == 0)
       {
		   cur_stage_time_expiry_flag = STATE_YES;		   			
	   }
    }  
 }  



void Init_ADC_Channel(const unsigned int adc_channel)
{
	unsigned int adc_channel_reg = adc_channel << 2;
    /* Fosc/32, corresponding channel, not yet start ADC, enable ADC channel */	
	ADCON0 = adc_channel_reg | 0x80 | 0x01;
	/* right justified, Vref+ and Vref- as internal */
    ADCON1 = 0x80; 
}
unsigned long Read_ADC_Channel(unsigned int adc_channel)
{
	unsigned int adc_value_least_byte, adc_value_most_byte;
	unsigned long adc_value_channel;
	/* Init_ADC_Channel should be called at the time of reading that ADC channel,
	if Init_ADC_Channel called at reset point ie before while(1), then last ADC channel that called
	Init_ADC_Channel() will effect ADC conversion, other ADC channels that called Init_ADC_Channel()
    before last channel's call on Init_ADC_Channel(), will not do ADC conversion */
	Init_ADC_Channel(adc_channel);
	/* After the analog input channel is selected (or changed), an A/D acquisition must be done before the conversion
		     can be started, minimum A/D acquisition time = 5us */
	Delay_Time_ByCount(ADC_ACQUIST_TIME_IN_CNT);
	/* Start ADC conversion for specific channel*/
	GO = 1;
	while(GO == 1);
	/* ADC conversion has completed*/
	adc_value_least_byte = ADRESL;
	adc_value_most_byte = ADRESH & 0x03u;
	switch(adc_value_most_byte)
	{
		case 0x00:
		  adc_value_channel = adc_value_least_byte;
		  break;
		case 0x01:
          adc_value_channel = adc_value_least_byte + 256u;
		  break;	
		case 0x02:
          adc_value_channel = adc_value_least_byte + 512u;
		  break; 
		case 0x03:
          adc_value_channel = adc_value_least_byte + 768u;
		  break; 
	}
	return adc_value_channel;
}
/*------------------------------------------------------------*
FUNCTION NAME  : 

DESCRIPTION    :
								
INPUT          : none

OUTPUT         : 

NOTE           : 

Func ID        : 29   
-*------------------------------------------------------------*/
void Encoded_To_Actual_Analog_Val_Calc(const unsigned long adc_value, const unsigned long full_scale_input_analog, \
 const unsigned min_input_analog, unsigned long *const analog_val_in_digital_int, unsigned long *const analog_val_in_digital_frac   )
{
	unsigned long remainder_val;	
	*analog_val_in_digital_int = (( full_scale_input_analog * adc_value) / (1024ul - 1))  + min_input_analog;
	remainder_val = ( full_scale_input_analog * adc_value) %(1024ul - 1 );
	*analog_val_in_digital_frac = ((remainder_val * 10) /(1024ul -1));
}
/*------------------------------------------------------------*
FUNCTION NAME  : 

DESCRIPTION    :
								
INPUT          : none

OUTPUT         : 

NOTE           : 

Func ID        : 29   
-*------------------------------------------------------------*/
void Encoded_To_Percent_Calc(const unsigned long adc_value, unsigned int *const percent_int, unsigned int *const percent_frac )
{
	unsigned int remainder_val;
	unsigned long temp_percent_int;
	temp_percent_int =  (100 * adc_value); 
	*percent_int = (100 * adc_value) / (1024ul - 1);
	remainder_val = temp_percent_int % (1024ul - 1 );
	*percent_frac = (remainder_val * 10) /(1024ul - 1);
}
/*------------------------------------------------------------*
FUNCTION NAME  : 

DESCRIPTION    :
								
INPUT          : none

OUTPUT         : 

NOTE           : 

Func ID        : 15  
-*------------------------------------------------------------*/
void Run_Timer1(const unsigned int set_timer1_mode )
{
     Stop_Timer1();
     /*internal timer1 clock  with 1:1 prescale,Timer1 counts when gate(T1G) is high 
     Timer1 counting is controlled by the Timer1 Gate function and no gate input is feed
     and enable timer1*/
	 
	  TMR1H = 0;
      TMR1L = 0;
	  TMR1IF = 0;
	  timer1_mode = set_timer1_mode;
	  /* for T1G gate based  timer 1 running control, Timer1 runs when T1G is high. If T1G is low, timer1 pauses counting */
	 // T1CON =0xC5; 

	 /*internal timer1 clock  with 1:1 prescale, gate(T1G) control for Timer1 is disabled  and enable timer1 and Timer1 runs */
      T1CON =0x85;   
      prescale_timer1 = 0x01;
      prescale_shift_timer1= 0;
      Prescale_Timer1();
      timer1_init = (65536UL) - (INC1/prescale_timer1); 
      TMR1H = timer1_init / 256UL;
      TMR1L = timer1_init % 256UL; 
      num_calls_timer1 = 0; 
      count_update_cur_stage_per_sec = 0; 
   
}
/*------------------------------------------------------------*
FUNCTION NAME  : 

DESCRIPTION    :
								
INPUT          : none

OUTPUT         : 

NOTE           : 

Func ID        : 16  
-*------------------------------------------------------------*/
void Stop_Timer1()
{
	if(timer1_mode != TMR1_OFF_STATE)
	{	
	   timer1_mode = TMR1_OFF_STATE;
	   T1CON = 0x80;
	}   
}
/*------------------------------------------------------------*
FUNCTION NAME  : 

DESCRIPTION    :
								
INPUT          : none

OUTPUT         : 

NOTE           : 

Func ID        : 17  
-*------------------------------------------------------------*/
void Prescale_Timer1()
{
   if(T1CKPS0 == 1)
   {
      prescale_shift_timer1 |= 0x01;           
   }
   if(T1CKPS1 == 1)
   {
     prescale_shift_timer1 |= 0x02;
   }  
   prescale_timer1 = prescale_timer1  << prescale_shift_timer1;                                                      
}

/*------------------------------------------------------------*
FUNCTION NAME  : 

DESCRIPTION    :
								
INPUT          : none

OUTPUT         : 

NOTE           : 

Func ID        : 29   
-*------------------------------------------------------------*/
void Goto_XY_LCD_Disp(const unsigned int start_line_num, const unsigned int start_col_num)
{
	/* max 4 lines and 20 columns */
	/* for us, lcd line starts from 1, but for uC, line starts from 0 */
	/* for us, lcd col starts from 1, but for uC, col starts from 0 */
    unsigned int start_line_lcd = start_line_num - 1, start_col_lcd = start_col_num - 1, error_disp_start_loc; 
		
   if(start_line_num <= CONFIGURE_MAX_NUM_LINES && start_col_num <= CONFIGURE_MAX_NUM_COLS )
   {
      switch(start_line_num)
	 {
		 case NUM_LINE1:
		   cur_disp_lcd_loc = BEGIN_LOC_LINE1;
		   break;
		 case NUM_LINE2:
		   cur_disp_lcd_loc = BEGIN_LOC_LINE2;
		   break;
		 case NUM_LINE3:
		   cur_disp_lcd_loc = BEGIN_LOC_LINE3;
		   break;
		  case NUM_LINE4:
		   cur_disp_lcd_loc = BEGIN_LOC_LINE4;
		   break;		 
	 }	
      cur_disp_lcd_loc = cur_disp_lcd_loc + start_col_lcd;
      Write_LCD_Command(cur_disp_lcd_loc);       
   }
   else
   {
	   /* error due to invalid lcd DISP loc  */    
		        		
   }	   
}
void LCD_Const_Disp()
{
	const char vol_signal_rep_disp[] = " Vol", reseveriour_msg_disp[] = "R:", vessel_msg_disp[] = "V:";	
	
	
	Goto_XY_LCD_Disp(RSV_CHS_LINE_NUM, NUM_COL1);
	Data_Str_Disp_LCD(reseveriour_msg_disp);	
	Goto_XY_LCD_Disp(RSV_CHS_LINE_NUM, RSV_CMT_PERCENT_DECPT_COL_NUM);
	Write_LCD_Data('.');
	Goto_XY_LCD_Disp(RSV_CHS_LINE_NUM, RSV_SAND_PERCENT_DECPT_COL_NUM);
	Write_LCD_Data('.');	
	Goto_XY_LCD_Disp(RSV_CHS_LINE_NUM, RSV_WATER_PERCENT_DECPT_COL_NUM);
	Write_LCD_Data('.');
	Goto_XY_LCD_Disp(RSV_CHS_LINE_NUM, RSV_CHS_PERCENT_MSG_COL_NUM);
	Data_Str_Disp_LCD("%");
	
    Goto_XY_LCD_Disp(VSL_CHS_LINE_NUM, NUM_COL1);
	Data_Str_Disp_LCD(vessel_msg_disp);		
	Goto_XY_LCD_Disp(VSL_CHS_LINE_NUM, VSL_CHS_VOLUME_MSG_COL_NUM);
    Data_Str_Disp_LCD(vol_signal_rep_disp);
	
	
	
	
}

 /*------------------------------------------------------------*
FUNCTION NAME  : 

DESCRIPTION    :
								
INPUT          : none

OUTPUT         : 

NOTE           : 

Func ID        : 21   
-*------------------------------------------------------------*/
void LCD_Init()
{
    Write_LCD_Command(0x30);
    Write_LCD_Command(0x30);
    Write_LCD_Command(0x30);
    Write_LCD_Command(0x38);
    Write_LCD_Command(0x01);
    Write_LCD_Command(0x0E); //insert cursor on at cur_disp_lcd_loc
	//Write_LCD_Command(0x0C);
    Write_LCD_Command(0x06);                                       
}     

/*------------------------------------------------------------*
FUNCTION NAME  : 

DESCRIPTION    :
								
INPUT          : none

OUTPUT         : 

NOTE           : 

Func ID        : 23   
-*------------------------------------------------------------*/
  void Write_LCD_Command (unsigned int cmd)
  {
	  unsigned long int  time_delay = MAX_COUNT_DELAY_TIME_LCDPULSE;
	  
       RW_PIN = 0;
       RS_PIN = 0; 
       LCD_PORT = cmd;
      // LCD_Pulse();
	  EN_PIN = 1;
	  while(time_delay--);
    // Delay_Time_ByCount(MAX_COUNT_DELAY_TIME_LCDPULSE);
      EN_PIN = 0;
	  time_delay = MAX_COUNT_DELAY_TIME_LCDPULSE;
	  while(time_delay--);
   //  Delay_Time_ByCount(MAX_COUNT_DELAY_TIME_LCDPULSE);
 }
 /*------------------------------------------------------------*
FUNCTION NAME  : 

DESCRIPTION    :
								
INPUT          : none

OUTPUT         : 

NOTE           : 

Func ID        : 24   
-*------------------------------------------------------------*/
 void Write_LCD_Data(const char ch)
{
	unsigned long int  time_delay = MAX_COUNT_DELAY_TIME_LCDPULSE;
	// performance degraded if many data is written to LCD, to check if write loc is within avail disp loc
	// if(lcd_avail_loc_within_limit == STATE_YES) 
	{	
      RW_PIN = 0;
      RS_PIN = 1;
      LCD_PORT =ch;
     // LCD_Pulse();
      EN_PIN = 1;
	  while(time_delay--);
    // Delay_Time_ByCount(MAX_COUNT_DELAY_TIME_LCDPULSE);
     EN_PIN = 0;
	 time_delay = MAX_COUNT_DELAY_TIME_LCDPULSE;
	  while(time_delay--);
   //  Delay_Time_ByCount(MAX_COUNT_DELAY_TIME_LCDPULSE);	 
	}
}
/*------------------------------------------------------------*
FUNCTION NAME  : 

DESCRIPTION    :
								
INPUT          : none

OUTPUT         : 

NOTE           : 

Func ID        : 25   
-*------------------------------------------------------------*/

void Data_Str_Disp_LCD(const char *char_ptr)
{ 
       while(*char_ptr)
       {
            Write_LCD_Data(*(char_ptr++));
       }
     
}
/*------------------------------------------------------------*
FUNCTION NAME  : 

DESCRIPTION    :
								
INPUT          : none

OUTPUT         : 

NOTE           : 

Func ID        : 27  
-*------------------------------------------------------------*/
void Delay_Time_ByCount( unsigned int time_delay)
{
       while(time_delay--);
}
/*------------------------------------------------------------*
FUNCTION NAME  : 

DESCRIPTION    :
								
INPUT          : none

OUTPUT         : 

NOTE           : 

Func ID        : 26  
-*------------------------------------------------------------*/
void Data_Num_Disp_LCD(const unsigned int lcd_datanum_disp_format, const unsigned long lcd_disp_data_int)
{
    unsigned int tens_thousand_digit,thousands_digit, hundreds_digit,tens_digit, unit_digit;
    unsigned long num = lcd_disp_data_int;
    char num_data[] ={'0','1','2','3','4','5','6','7','8','9'};  
	char hex_data[] ={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'}; 
    switch(lcd_datanum_disp_format)
	{
	  case DISP_FLAG_NUM_DIGIT5: 
		  num =  lcd_disp_data_int % 100000;
		  tens_thousand_digit = (unsigned int)(num / (unsigned long)(10000));
          Write_LCD_Data(num_data[tens_thousand_digit]);
	  case DISP_FLAG_NUM_DIGIT4:
	      num = lcd_disp_data_int % 10000;
	      thousands_digit = (unsigned int)(num / (unsigned long)(1000));
	      Write_LCD_Data(num_data[thousands_digit]); 
	  case DISP_FLAG_NUM_DIGIT3: 
		  num = lcd_disp_data_int % 1000;
	      hundreds_digit = (unsigned int) (num / (unsigned long) (100));
	      Write_LCD_Data(num_data[hundreds_digit]);
	  case DISP_FLAG_NUM_DIGIT2:
          num = lcd_disp_data_int % 100;
          tens_digit = (unsigned int) (num / 10);
          Write_LCD_Data(num_data[tens_digit]); 		  
	  case DISP_FLAG_NUM_DIGIT1:
	     unit_digit = (unsigned int) (lcd_disp_data_int % 10);
         Write_LCD_Data(num_data[unit_digit]); 
	  break;
	  case DISP_FLAG_HEX_DIGIT4:
	      /*  ( 16 * 16 * 16 *16 )  = 0 as divide by zero warning */
	      //num = lcd_disp_data_int % ( 16 * 16 * 16 *16 );
          thousands_digit = (num / (16 * 16 * 16));
	      Write_LCD_Data(hex_data[thousands_digit]);
	  case DISP_FLAG_HEX_DIGIT3:
	      num = lcd_disp_data_int %(unsigned long)(16 * 16 * 16);
	      hundreds_digit = (unsigned int) (num / (unsigned long) (16 * 16));
	      Write_LCD_Data(hex_data[hundreds_digit]);
	  case DISP_FLAG_HEX_DIGIT2:
	      num = lcd_disp_data_int %(unsigned long)(16 * 16);
          tens_digit = (unsigned int) (num / 16);
          Write_LCD_Data(hex_data[tens_digit]);
	  case DISP_FLAG_HEX_DIGIT1: 
	      unit_digit = (unsigned int) (lcd_disp_data_int % 16);
          Write_LCD_Data(hex_data[unit_digit]);    
	  break;
	  default:
	     /* Warning invalid lcd_disp flag */
	    ;
	}   	
}
