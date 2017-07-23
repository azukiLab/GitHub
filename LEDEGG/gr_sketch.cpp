#include <Arduino.h>
#include <RTC.h>
// ------------------------------------------------------ 
#define EGG_TYPE	2	// =1卓登, =2雄登
// ------------------------------------------------------ 
#define USE_SW2		0
#define USE_STANDBY	1
// ------------------------------------------------------ 
#define LED_R		22	// pin for COTTON LED
#define LED_G		23	// pin for COTTON LED
#define LED_B		24	// pin for COTTON LED
#define LED_PIN		3	// pin for LEDs WS2812B
#if (EGG_TYPE == 2)
 #define CATTERING	500	// チャタリング除去時間(ms)
 #define LED_NUM	3	// LEDの数
 #define A6_VAL		900	// A6タッチセンサ閾値
 #define AVERAGE	5	// A6タッチセンサ平均化
 #define A6_Repeat	3	// A6タッチセンサ連続回数
#else
 #define CATTERING	500	// チャタリング除去時間(ms)
 #define LED_NUM	4	// LEDの数
 #define A6_VAL		980	// A6タッチセンサ閾値
#endif

// ------------------------------------------------------ 
#define LED_PINREG		P1.BIT.bit6 // register for rapid access to pin
#define LEDPIN_HIGH     LED_PINREG = 1
#define LEDPIN_LOW		LED_PINREG = 0
// ------------------------------------------------------ 
typedef enum {
	MODE_OFF=0,		// 省電力
#if (EGG_TYPE == 2)
	MODE_DAYLIGHT,	// 昼光色
	MODE_GRADATION,	// グラデーション
	MODE_BULB,		// 電球色
#else
	MODE_BULB,		// 電球色
	MODE_DAYLIGHT,	// 昼光色
	MODE_GRADATION,	// グラデーション
	MODE_R_Nm0,		// ルーレットN個回転
	MODE_R_End,		// ルーレット0個回転
#endif
	MODE_MAX
} EN_MODE;
typedef enum {
	LEDOFF=0,	// 消灯
	BULB,		// 電球色
	DAYLIGHT,	// 昼光色
	RED,		// 赤
	ORANGE,		// 橙
	GREEN,		// 緑
	BLUE,		// 青
	PURPLE,		// 紫
	COLOR_MAX
} EN_COLOR;
#define WHITE	DAYLIGHT
const unsigned char cRGB[COLOR_MAX][3] = {
	//  R,   G,   B
	{   0,   0,   0},	// 消灯
	{0xF3,0x98,0x00},	// 電球色
	{0xFF,0xFF,0xFF},	// 昼光色
	{0xE6,0x00,0x12},	// 赤
	{0xF3,0x98,0x00},	// 橙
	{0x00,0x99,0x44},	// 緑
	{0x1D,0x20,0x88},	// 藍
	{0x92,0x07,0x83}	// 紫
};
// ------------------------------------------------------ 
typedef struct {
	EN_MODE nextMode;	// 次の状態
	const int timeout;	// タイムアウト(分)
	EN_MODE alarmMode;	// タイムアウト後の状態
} T_MODECFG;
const T_MODECFG cfg[MODE_MAX] = {
#if (EGG_TYPE == 2)
/*MODE_OFF*/		{MODE_DAYLIGHT,	0,	MODE_OFF		},
/*MODE_DAYLIGHT*/	{MODE_GRADATION,5,	MODE_OFF		},
/*MODE_GRADATION*/	{MODE_BULB,		5,	MODE_OFF		},
/*MODE_BULB*/		{MODE_DAYLIGHT,	5,	MODE_OFF		}
#else
/*MODE_OFF*/		{MODE_BULB,		0,	MODE_OFF		},
/*MODE_BULB*/		{MODE_DAYLIGHT,	10,	MODE_OFF		},
/*MODE_DAYLIGHT*/	{MODE_GRADATION,10,	MODE_OFF		},
/*MODE_GRADATION*/	{MODE_R_Nm0,	5,	MODE_OFF		},
/*MODE_R_Nm0*/		{MODE_R_End,	1,	MODE_GRADATION	},
/*MODE_R_End*/		{MODE_OFF,		5,	MODE_OFF		}
#endif
};
// ------------------------------------------------------ 
volatile int gMode = (EN_MODE)MODE_OFF;
volatile boolean gModeChange = false;
volatile unsigned char gCurrent[LED_NUM][3] = {{0}};
volatile int gColor = (int)LEDOFF;
static double t[3] = {0.0, 10.0, 20.0};
RTC_TIMETYPE time;
volatile int gSensor = A6_VAL;
volatile int gA6Repeat[2] = {0, 0};
// ------------------------------------------------------ 
// Wait routine to construct the signal for LEDs.
// Needed to transfer the signal during DI because tight timing.
#define DI()	asm("di")
#define EI()	asm("ei")
#define LED_ON_WAIT0   NOP();NOP();NOP();NOP();NOP();NOP();
#define LED_OFF_WAIT0  NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
#define LED_ON_WAIT1   NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
#define LED_OFF_WAIT1  NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
void sendLedData(uint8_t red, uint8_t green, uint8_t blue)
{
    if (green & 0x80) {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (green & 0x40) {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (green & 0x20) {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (green & 0x10) {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (green & 0x08) {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (green & 0x04) {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (green & 0x02) {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (green & 0x01) {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send

    if (red & 0x80)   {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (red & 0x40)   {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (red & 0x20)   {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (red & 0x10)   {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (red & 0x08)   {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (red & 0x04)   {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (red & 0x02)   {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (red & 0x01)   {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send

    if (blue & 0x80)  {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (blue & 0x40)  {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (blue & 0x20)  {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (blue & 0x10)  {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (blue & 0x08)  {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (blue & 0x04)  {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (blue & 0x02)  {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
    if (blue & 0x01)  {LEDPIN_HIGH; LED_ON_WAIT1; LEDPIN_LOW; LED_OFF_WAIT1;}	// on data send
	else			  {LEDPIN_HIGH; LED_ON_WAIT0; LEDPIN_LOW; LED_OFF_WAIT0;}	// off data send
}
// ------------------------------------------------------ 
volatile unsigned long gNow=0, gPrev=0;
void IntHdrCyclic1ms(unsigned long now_ms)
{
	gNow = now_ms;	// 経過時間更新
	if(gPrev > gNow)	// gNowが一周した
	{
		gPrev = 0;
	}
}
// ------------------------------------------------------ 
void set_next_alarm()
{
#if USE_STANDBY
	if(cfg[gMode].timeout > 0)
	{
		time.hour	= 0;	// 時
		time.min	= 0;	// 分
		time.second	= 0;	// 秒
		rtc_set_time(&time);// 時刻設定
		rtc_set_alarm_time(time.hour, time.min+cfg[gMode].timeout);	// タイムアウト時刻設定
		rtc_alarm_on();
	}
	else
	{
		rtc_alarm_off();
	}
#endif
}
#if USE_STANDBY
void alarm_handler(void)
{
	gMode = cfg[gMode].alarmMode;	// タイムアウト後の状態に遷移
	set_next_alarm();	// 次のタイムアウト時刻設定
}
#endif
// ------------------------------------------------------ 
void setRequestModeChange()
{
	gModeChange = true;	// モード変更要求
}
int ReadSens_and_Condition(){
	int i;
	int sval;

	sval = 0;
	for (i = 0; i < AVERAGE; i++){
		sval += analogRead(A6);
	}
	sval /= AVERAGE;
//	sval >>= 2;    // 8ビット化(0 - 255)
	return sval;
}
boolean senseRequestModeChange()
{
	boolean ret = false;

	if((gNow - gPrev) >= CATTERING)	// CATTERING時間以内の連続入力を無視する
	{
#if (EGG_TYPE == 2)
		gSensor = ReadSens_and_Condition();
#else
		gSensor = analogRead(A6);
#endif
		if(gSensor < A6_VAL){
			gPrev = gNow;
			ret = true;
		}
		else{
			ret = false;
		}
#if (EGG_TYPE == 2)
		if(gSensor < 1000){
			if(++gA6Repeat[0] > A6_Repeat)
			{
				gPrev = gNow;
				ret = true;
			}
		}
		else
		{
			if(++gA6Repeat[1] > (3*A6_Repeat))
			{
				gA6Repeat[0] = 0;
				gA6Repeat[1] = 0;
				}
		}
		if(ret)
		{
			gA6Repeat[0] = 0;
			gA6Repeat[1] = 0;
		}
#endif
#if 0
		Serial.print(gMode);
		Serial.print(", sensor = ");
		Serial.print(gSensor);
		Serial.print(", gA6Repeat[0] = ");
		Serial.println(gA6Repeat[0]);
#endif
	}

	return ret;
}
void waitRequestModeChange()
{
	// 1ms周期割り込み解除
	detachIntervalTimerHandler();

	setPowerManagementMode(PM_SNOOZE_MODE, 0, A6_VAL);	// A6入力0〜900くらいでwakeup
	gSensor = analogRead(A6); // into snooze until touch 
	setPowerManagementMode(PM_NORMAL_MODE);	// 解除

	// 1ms周期割り込み設定
	attachIntervalTimerHandler(IntHdrCyclic1ms);
#if 0
	Serial.print(gMode);
	Serial.print(", sensor = ");
	Serial.println(gSensor);
#endif
}
// ------------------------------------------------------ 
#if USE_SW2
void Int0_colorChange(void)
{
	if((gNow - gPrev) >= CATTERING)
	{
		gPrev = gNow;
		gMode = MODE_OFF;
	}
}
#endif
// ------------------------------------------------------ 
void setup(void)
{
	int i;

	Serial.begin(9600);
	Serial.println("LED Egg");
	Serial.flush();

	// 初回値0になるみたいなのでダミーリードが必要っぽい
	analogRead(A6);

	gMode = MODE_DAYLIGHT;
	gModeChange = false;
	gColor = LEDOFF;
	gSensor = A6_VAL;
	gNow = 0;
	gPrev = 0;

	// LED点灯
	pinMode(LED_PIN, OUTPUT);
	DI();
	for (i = 0; i < LED_NUM; i++){
		sendLedData(cRGB[WHITE][0], cRGB[WHITE][1], cRGB[WHITE][2]);
	}
	EI();
	pinMode(LED_R, OUTPUT);	// analogWrite()では不要
	pinMode(LED_G, OUTPUT);	// analogWrite()では不要
	pinMode(LED_B, OUTPUT);	// analogWrite()では不要
	digitalWrite(LED_R, LOW);
	digitalWrite(LED_G, LOW);
	digitalWrite(LED_B, LOW);

#if USE_STANDBY
	// タイムアウト監視
	rtc_init();
	time.year		= 2016;
	time.mon		= 9;
	time.day		= 1;
	time.weekday	= RTC_ALARM_THURSDAY;
	time.hour		= 0;
	time.min		= 0;
	time.second		= 0;
	rtc_set_time(&time);
	rtc_attach_alarm_handler(alarm_handler);
	rtc_set_alarm_time(time.hour, time.min+cfg[gMode].timeout);
	rtc_alarm_on();
#endif

#if USE_SW2
	// SW2入力
	pinMode(2, INPUT_PULLUP);
	attachInterrupt(0, Int0_colorChange, FALLING);
#endif
}
// ------------------------------------------------------ 
void loop(void)
{
	int i;
	unsigned int N = LED_NUM;

	// 各モードの処理 ここから
	switch(gMode)
	{
	// 省電力-------------------------------------------------- 
	default:
		/* FALLTHROUGH */
	case MODE_OFF:
		// LED off
		for (i = 0; i < LED_NUM; i++){
			gCurrent[i][0] = cRGB[LEDOFF][0];
			gCurrent[i][1] = cRGB[LEDOFF][1];
			gCurrent[i][2] = cRGB[LEDOFF][2];
		}
		DI();
		for (i = 0; i < LED_NUM; i++){
			sendLedData(gCurrent[i][0], gCurrent[i][1], gCurrent[i][2]);
		}
		EI();
		digitalWrite(LED_R, HIGH);
		digitalWrite(LED_G, HIGH);
		digitalWrite(LED_B, HIGH);

		delay(CATTERING);
		// ここでスタンバイに遷移、A6入力で復帰
		waitRequestModeChange();
		setRequestModeChange();		// モード変更要求設定
		break;
	// 電球色-------------------------------------------------- 
	case MODE_BULB:
		for (i = 0; i < LED_NUM; i++){
			gCurrent[i][0] = cRGB[BULB][0];
			gCurrent[i][1] = cRGB[BULB][1];
			gCurrent[i][2] = cRGB[BULB][2];
		}
		DI();
		for (i = 0; i < LED_NUM; i++){
			sendLedData(gCurrent[i][0], gCurrent[i][1], gCurrent[i][2]);
		}
		EI();
		digitalWrite(LED_R, HIGH);
		digitalWrite(LED_G, HIGH);
		digitalWrite(LED_B, HIGH);

		// ここでスタンバイに遷移、A6入力で復帰
		waitRequestModeChange();
		setRequestModeChange();		// モード変更要求設定
//		delay(CATTERING);
		break;
	// 昼光色-------------------------------------------------- 
	case MODE_DAYLIGHT:
		for (i = 0; i < LED_NUM; i++){
			gCurrent[i][0] = cRGB[DAYLIGHT][0];
			gCurrent[i][1] = cRGB[DAYLIGHT][1];
			gCurrent[i][2] = cRGB[DAYLIGHT][2];
		}
		DI();
		for (i = 0; i < LED_NUM; i++){
			sendLedData(gCurrent[i][0], gCurrent[i][1], gCurrent[i][2]);
		}
		EI();
		digitalWrite(LED_R, HIGH);
		digitalWrite(LED_G, HIGH);
		digitalWrite(LED_B, HIGH);

		// ここでスタンバイに遷移、A6入力で復帰
		waitRequestModeChange();
		setRequestModeChange();		// モード変更要求設定
//		delay(CATTERING);
		break;
	// グラデーション-------------------------------------------------- 
	case MODE_GRADATION:
#if (EGG_TYPE == 1)
		gSensor = analogRead(A6); // into snooze until touch 
		if(gSensor > A6_VAL)
		{
#endif
			t[0] = fmod(t[0] + (2 * M_PI / 1000 * 10), 60.0 * M_PI);
			t[1] = fmod(t[1] + (2 * M_PI / 1000 * 10), 60.0 * M_PI);
			t[2] = fmod(t[2] + (2 * M_PI / 1000 * 10), 60.0 * M_PI);
#if (EGG_TYPE == 1)
			for (i = 0; i < LED_NUM; i++){
				gCurrent[i][0] = int(255.0 * pow(0.5 - 0.49 * cos(t[0] / 2.0), 2.0));
				gCurrent[i][1] = int(255.0 * pow(0.5 - 0.49 * cos(t[0] / 5.0), 2.0));
				gCurrent[i][2] = int(255.0 * pow(0.5 - 0.49 * cos(t[0] / 3.0), 2.0));
			}
#else
			for (i = 0; i < LED_NUM; i++){
				gCurrent[i][0] = int(255.0 * pow(0.5 - 0.49 * cos(t[i] / 2.0), 2.0));
				gCurrent[i][1] = int(255.0 * pow(0.5 - 0.49 * cos(t[i] / 5.0), 2.0));
				gCurrent[i][2] = int(255.0 * pow(0.5 - 0.49 * cos(t[i] / 3.0), 2.0));
			}
#endif
			DI();
			for (i = 0; i < LED_NUM; i++){
				sendLedData(gCurrent[i][0], gCurrent[i][1], gCurrent[i][2]);
			}
			EI();
	
			delay(20);	// LED色変更速度調整
#if (EGG_TYPE == 1)
		}
		else
		{
			if((gNow - gPrev) >= CATTERING)	// CATTERING時間以内の連続入力を無視する
			{
				gPrev = gNow;
				setRequestModeChange();	// モード変更要求設定
//				delay(CATTERING);
			}
		}
#if 0
		Serial.print(gMode);
		Serial.print(", sensor = ");
		Serial.println(gSensor);
#endif

#else
		if(senseRequestModeChange())
		{
			setRequestModeChange();	// モード変更要求設定
		}
#endif
		break;
	// ルーレット-------------------------------------------------- 
#if (EGG_TYPE == 1)
	case MODE_R_End:// ルーレット停止
		N = 0;
		/* FALLTHROUGH */
	case MODE_R_Nm0:// ルーレットN個回転
		gSensor = analogRead(A6); // into snooze until touch 
		if(gSensor > A6_VAL)
		{
			if(++gColor >= COLOR_MAX)
			{
				gColor = WHITE;
			}
			for (i = 0; i < N; i++){
				gCurrent[i][0] = cRGB[gColor][0];
				gCurrent[i][1] = cRGB[gColor][1];
				gCurrent[i][2] = cRGB[gColor][2];
			}
			DI();
			for (i = 0; i < LED_NUM; i++){
				sendLedData(gCurrent[i][0], gCurrent[i][1], gCurrent[i][2]);
			}
			EI();

			delay(330);	// ルーレット回転速度調整(ms)
		}
		else
		{
			if((gNow - gPrev) >= CATTERING)	// CATTERING時間以内の連続入力を無視する
			{
				gPrev = gNow;
				setRequestModeChange();	// モード変更要求設定
				delay(CATTERING);
			}
		}
		break;
#endif
	}
	// 各モードの処理 ここまで

	if(gModeChange)	// モード変更要求あり?
	{
		gModeChange = false;	// モード変更要求 解除

		delay(CATTERING);
		gMode = cfg[gMode].nextMode;
		set_next_alarm();	// 次のタイムアウト時刻設定
	}
}
