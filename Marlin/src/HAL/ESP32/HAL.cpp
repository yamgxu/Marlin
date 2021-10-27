/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#ifdef ARDUINO_ARCH_ESP32

#include "../../inc/MarlinConfig.h"

#include <soc/adc_channel.h>
#include <soc/rtc.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <HardwareSerial.h>
#include "USB.h"

#if ENABLED(WIFISUPPORT)
  #include "wifi.h"
  #if ENABLED(OTASUPPORT)
    #include "ota.h"
  #endif
  #if ENABLED(WEBSUPPORT)
    #include "spiffs.h"
    #include "web.h"
  #endif
#endif

#if ENABLED(ESP3D_WIFISUPPORT)
  DefaultSerial1 MSerial0(false, Serial2Socket);
#endif

// ------------------------// ------------------------
// Externs//外部人员
// ------------------------// ------------------------

portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

// ------------------------// ------------------------
// Local defines//局部定义
// ------------------------// ------------------------

#define V_REF 1100

// ------------------------// ------------------------
// Public Variables//公共变量
// ------------------------// ------------------------

// DefaultSerial1 MSerial1(false, Serial);//DefaultSerial1 MSerial1（假，串行）；
// DefaultSerial2 MSerial2(false, Serial1);//DefaultSerial2 MSerial2（false，Serial1）；
DefaultSerialUSB MSerialUSB(false);

uint16_t HAL_adc_result;

// ------------------------// ------------------------
// Private Variables//私有变量
// ------------------------// ------------------------

esp_adc_cal_characteristics_t characteristics[ADC_ATTEN_MAX];
adc_atten_t attenuations[ADC1_CHANNEL_MAX] = {};
uint32_t thresholds[ADC_ATTEN_MAX];
volatile int numPWMUsed = 0,
             pwmPins[MAX_PWM_PINS],
             pwmValues[MAX_PWM_PINS];

// ------------------------// ------------------------
// Public functions//公共职能
// ------------------------// ------------------------

#if ENABLED(WIFI_CUSTOM_COMMAND)

  bool wifi_custom_command(char * const command_ptr) {
    #if ENABLED(ESP3D_WIFISUPPORT)
      return esp3dlib.parse(command_ptr);
    #else
      UNUSED(command_ptr);
      return false;
    #endif
  }

#endif

void HAL_init_board() {
  USB.productName("ESP32S2-USB");
  USB.begin();
    HardwareSerial Serial1(1);
    Serial.begin(115200);
    Serial.println("hello, world11");
  #if ENABLED(ESP3D_WIFISUPPORT)
    esp3dlib.init();
  #elif ENABLED(WIFISUPPORT)
    wifi_init();
    TERN_(OTASUPPORT, OTA_init());
    #if ENABLED(WEBSUPPORT)
      //spiffs_init();//spiffs_init（）；
      web_init();
    #endif
  #endif

  // ESP32 uses a GPIO matrix that allows pins to be assigned to hardware serial ports.//ESP32使用GPIO矩阵，允许将管脚分配给硬件串行端口。
  // The following code initializes hardware Serial1 and Serial2 to use user-defined pins//以下代码初始化硬件Serial1和Serial2以使用用户定义的管脚
  // if they have been defined.//如果它们已经被定义。

   #if defined(HARDWARE_SERIAL1_RX) && defined(HARDWARE_SERIAL1_TX)
     // HardwareSerial Serial1(1);//《硬件研究》系列1（1）；
     #ifdef TMC_BAUD_RATE  // use TMC_BAUD_RATE for Serial1 if defined//如果定义了串行1，则使用TMC_波特率
      // Serial.begin(TMC_BAUD_RATE, SERIAL_8N1, HARDWARE_SERIAL1_RX, HARDWARE_SERIAL1_TX);//串行。开始（TMC_波特率、串行_8N1、硬件_串行1_RX、硬件_串行1_TX）；
     #else  // use default BAUDRATE if TMC_BAUD_RATE not defined//如果未定义TMC_波特率，则使用默认波特率
      // Serial.begin(BAUDRATE, SERIAL_8N1, HARDWARE_SERIAL1_RX, HARDWARE_SERIAL1_TX);//串行。开始（波特率、串行8N1、硬件串行1\uRx、硬件串行1\uTx）；
     #endif
   #endif
   #if defined(HARDWARE_SERIAL2_RX) && defined(HARDWARE_SERIAL2_TX)
     HardwareSerial Serial2(2);
     #ifdef TMC_BAUD_RATE  // use TMC_BAUD_RATE for Serial1 if defined//如果定义了串行1，则使用TMC_波特率
       Serial2.begin(TMC_BAUD_RATE, SERIAL_8N1, HARDWARE_SERIAL2_RX, HARDWARE_SERIAL2_TX);
     #else  // use default BAUDRATE if TMC_BAUD_RATE not defined//如果未定义TMC_波特率，则使用默认波特率
       Serial2.begin(BAUDRATE, SERIAL_8N1, HARDWARE_SERIAL2_RX, HARDWARE_SERIAL2_TX);
     #endif
   #endif

  // Initialize the i2s peripheral only if the I2S stepper stream is enabled.//仅当i2s步进器流启用时，才初始化i2s外围设备。
  // The following initialization is performed after Serial1 and Serial2 are defined as//将Serial1和Serial2定义为
  // their native pins might conflict with the i2s stream even when they are remapped.//即使重新映射，其本机管脚也可能与i2s流冲突。
//  TERN_(I2S_STEPPER_STREAM, i2s_init());//TERN_（I2S_步进器_流，I2S_init（））；
}

void HAL_idletask() {
  #if ENABLED(WIFISUPPORT)
    TERN_(OTASUPPORT, OTA_handle());
    webSocketSerial.handle_flush();
  #endif

  TERN_(ESP3D_WIFISUPPORT, esp3dlib.idletask());
}

void HAL_clear_reset_source() { }

uint8_t HAL_get_reset_source() { return esp_reset_reason(); }

void HAL_reboot() {
        SERIAL_ECHO_MSG("HAL_rebootHAL_rebootHAL_rebootHAL_rebootHAL_reboot wb");

      ESP.restart(); }

void _delay_ms(int delay_ms) { delay(delay_ms); }

// return free memory between end of heap (or end bss) and whatever is current//返回堆端（或bss端）和任何当前值之间的可用内存
int freeMemory() { return ESP.getFreeHeap(); }

// ------------------------// ------------------------
// ADC//模数转换器
// ------------------------// ------------------------
#define ADC1_CHANNEL(pin) ADC1_GPIO ## pin ## _CHANNEL

// TODO clean this up to justt use ADC1_CHANNEL() -- not possible because of runtime get_channel()//要清除此问题，只需使用ADC1_通道（）--由于运行时get_通道（）的原因，这是不可能的
adc1_channel_t get_channel(int pin) {
  switch (pin) {
    case 1: return ADC1_CHANNEL(1);
    case 2: return ADC1_CHANNEL(2);
    case 3: return ADC1_CHANNEL(3);
    case 4: return ADC1_CHANNEL(4);
    case 5: return ADC1_CHANNEL(5);
    // case 39: return ADC1_CHANNEL(39);//情况39：返回ADC1_信道（39）；
    // case 36: return ADC1_CHANNEL(36);//情况36：返回ADC1_信道（36）；
    // case 35: return ADC1_CHANNEL(35);//情况35：返回ADC1_信道（35）；
    // case 34: return ADC1_CHANNEL(34);//情况34：返回ADC1_信道（34）；
    // case 33: return ADC1_CHANNEL(33);//情况33：返回ADC1_信道（33）；
    // case 32: return ADC1_CHANNEL(32);//情况32：返回ADC1_信道（32）；
  }
  return ADC1_CHANNEL_MAX;
}

void adc1_set_attenuation(adc1_channel_t chan, adc_atten_t atten) {
  if (attenuations[chan] != atten) {
    adc1_config_channel_atten(chan, atten);
    attenuations[chan] = atten;
  }
}

void HAL_adc_init() {
  // Configure ADC//配置ADC
  adc1_config_width(ADC_WIDTH_BIT_13);

  // Configure channels only if used as (re-)configuring a pin for ADC that is used elsewhere might have adverse effects//仅当用作（重新）配置其他地方使用的ADC引脚时配置通道可能会产生不利影响
  TERN_(HAS_TEMP_ADC_0, adc1_set_attenuation(get_channel(TEMP_0_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_ADC_1, adc1_set_attenuation(get_channel(TEMP_1_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_ADC_2, adc1_set_attenuation(get_channel(TEMP_2_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_ADC_3, adc1_set_attenuation(get_channel(TEMP_3_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_ADC_4, adc1_set_attenuation(get_channel(TEMP_4_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_ADC_5, adc1_set_attenuation(get_channel(TEMP_5_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_ADC_6, adc2_set_attenuation(get_channel(TEMP_6_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_ADC_7, adc3_set_attenuation(get_channel(TEMP_7_PIN), ADC_ATTEN_11db));
  TERN_(HAS_HEATED_BED, adc1_set_attenuation(get_channel(TEMP_BED_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_CHAMBER, adc1_set_attenuation(get_channel(TEMP_CHAMBER_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_COOLER, adc1_set_attenuation(get_channel(TEMP_COOLER_PIN), ADC_ATTEN_11db));
  TERN_(FILAMENT_WIDTH_SENSOR, adc1_set_attenuation(get_channel(FILWIDTH_PIN), ADC_ATTEN_11db));

  // Note that adc2 is shared with the WiFi module, which has higher priority, so the conversion may fail.//请注意，adc2与具有更高优先级的WiFi模块共享，因此转换可能会失败。
  // That's why we're not setting it up here.//这就是为什么我们不在这里设置它。

  // Calculate ADC characteristics (i.e., gain and offset factors for each attenuation level)//计算ADC特性（即每个衰减级别的增益和偏移因子）
  for (int i = 0; i < ADC_ATTEN_MAX; i++) {
    esp_adc_cal_characterize(ADC_UNIT_1, (adc_atten_t)i, ADC_WIDTH_BIT_13, V_REF, &characteristics[i]);

    // Change attenuation 100mV below the calibrated threshold//将衰减更改为低于校准阈值100mV
    // thresholds[i] = esp_adc_cal_raw_to_voltage(4095, &characteristics[i]);//阈值[i]=esp_adc_cal_raw_至_电压（4095，&特性[i]）；
  }
}

void HAL_adc_start_conversion(const uint8_t adc_pin) {
  const adc1_channel_t chan = get_channel(adc_pin);
  uint32_t mv;
  //esp_err_t err = esp_adc_cal_get_voltage((adc_channel_t)chan, &characteristics[attenuations[chan]], &mv);//esp_err_t err=esp_adc_cal_get_voltage（（adc_channel_t）chan，&特性[衰减[chan]，&mv）；

    uint32_t adcRead = analogRead(adc_pin);
    //SERIAL_ECHOPAIR("adcRead", adcRead);//序列回波对（“adcRead”，adcRead）；
    //SERIAL_ECHO("\r\n");//串行回波（“\r\n”）；
    double xadc = (adcRead +72.27488151 )/1.2664459692;
    //SERIAL_ECHO("xadc" );//串行回波（“xadc”）；
    //SERIAL_ECHOLN(xadc);//串行_-ECHOLN（xadc）；
    double vy = xadc/8191.0*3.6;
   // SERIAL_ECHO("vy" );//序列回波（“vy”）；
    //SERIAL_ECHOLN(vy);//序列号（vy）；
    double v47=3.6-vy;
    double rc=4.7/(v47/vy);
    double r=1.0/(1.0/rc-1.0/4.7);
    //print('测温电阻',1/(1/y-1/4.7));//打印（'测温电阻',1/（1/y-1/4.7））；
    //SERIAL_ECHO("r" );//序列回波（“r”）；
    //SERIAL_ECHOLN(r);//序列号_-ECHOLN（r）；
    double adc=1024/(4.7/r+1 ); //还原 adc//还原 模数转换器
    //SERIAL_ECHO("adc" );//串行回波（“adc”）；
    //SERIAL_ECHOLN(adc);//串行接口（adc）；
    //SERIAL_ECHO("\r\n");//串行回波（“\r\n”）；
    HAL_adc_result = (uint16_t )(adc);

   //  SERIAL_ECHO("HAL_adc_result" );//串行回波（“HAL_adc_结果”）；
   //SERIAL_ECHOLN(HAL_adc_result);//串行回波（HAL adc结果）；
  // Change the attenuation level based on the new reading//根据新读数更改衰减级别
  adc_atten_t atten;
  if (mv < 750)
    atten = ADC_ATTEN_DB_0;
  else if (mv > 750 && mv < 1050)
    atten = ADC_ATTEN_DB_2_5;
  else if (mv > 1050 && mv < 1300)
    atten = ADC_ATTEN_DB_6;
  else if (mv > 1300)
    atten = ADC_ATTEN_DB_11;
  else return;

  adc1_set_attenuation(chan, atten);
}



void digitalWrite1(pin_t pin, int value) {

       // SERIAL_ECHO_MSG("analogWrite");
       // SERIAL_ECHO(int(pin));
       // SERIAL_ECHO(value);
       // SERIAL_ECHO_MSG("analogWrite ok");
      //if(pin == 2 ){
      //   if(value>0){
      //       value=0;
      //   }else{
      //       value=255;
      //   }
      //}
      digitalWrite(pin, value);
}
void analogWrite(pin_t pin, int value) {

      if(pin == 2 ){
          if(value>0){
              value=0;
          }else{
              value=255;
          }
       }
  // Use ledc hardware for internal pins//内部引脚使用ledc硬件
  if (pin < 34) {
    static int cnt_channel = 1, pin_to_channel[40] = { 0 };
    if (pin_to_channel[pin] == 0) {
      ledcAttachPin(pin, cnt_channel);
      ledcSetup(cnt_channel, 490, 8);
      ledcWrite(cnt_channel, value);
      pin_to_channel[pin] = cnt_channel++;
    }
    ledcWrite(pin_to_channel[pin], value);
    return;
  }

  int idx = -1;

  // Search Pin//搜索Pin码
  for (int i = 0; i < numPWMUsed; ++i)
    if (pwmPins[i] == pin) { idx = i; break; }

  // not found ?//找不到？
  if (idx < 0) {
    // No slots remaining//没有剩余的插槽
    if (numPWMUsed >= MAX_PWM_PINS) return;

    // Take new slot for pin//取新的插针槽
    idx = numPWMUsed;
    pwmPins[idx] = pin;
    // Start timer on first use//首次使用时启动计时器
    if (idx == 0) {
                HAL_timer_start(PWM_TIMER_NUM, PWM_TIMER_FREQUENCY);
    }

    ++numPWMUsed;
  }

  // Use 7bit internal value - add 1 to have 100% high at 255//使用7bit内部值-添加1，使其在255处具有100%高
  pwmValues[idx] = (value + 1) / 2;
}

// Handle PWM timer interrupt//处理PWM定时器中断
HAL_PWM_TIMER_ISR() {
      SERIAL_ECHO_MSG("HAL_PWM_TIMER_ISR");
      return;
  HAL_timer_isr_prologue(PWM_TIMER_NUM);

  static uint8_t count = 0;

  for (int i = 0; i < numPWMUsed; ++i) {
    if (count == 0)                   // Start of interval//间隔开始
      WRITE(pwmPins[i], pwmValues[i] ? HIGH : LOW);
    else if (pwmValues[i] == count)   // End of duration//期限结束
      WRITE(pwmPins[i], LOW);
  }

  // 128 for 7 Bit resolution//128表示7位分辨率
  count = (count + 1) & 0x7F;

  HAL_timer_isr_epilogue(PWM_TIMER_NUM);
}

#endif // ARDUINO_ARCH_ESP32//ARDUINO_ARCH_ESP32
