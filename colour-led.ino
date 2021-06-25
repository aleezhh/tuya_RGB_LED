#include <TuyaWifi.h>
#include <SoftwareSerial.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define PIXEL_PIN    6  // Digital IO pin connected to the NeoPixels.

#define PIXEL_COUNT 30  // Number of NeoPixels

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

boolean oldState = HIGH;
int     mode     = 0;    // Currently-active animation mode, 0-9


TuyaWifi my_device;
SoftwareSerial DebugSerial(8,9);
unsigned char pid[] = {"eduygmimidcelz7m"};//*********处替换成涂鸦IoT平台自己创建的产品的PID
unsigned char mcu_ver[] = {"1.0.2"};
unsigned long last_time = 0;
unsigned char led_state = 0;
int key_pin = 7;     // Connect network button pin

//开关(可下发可上报)
//备注:
#define DPID_SWITCH_LED 20
//模式(可下发可上报)
//备注:
#define DPID_WORK_MODE 21
//亮度值(可下发可上报)
//备注:
#define DPID_BRIGHT_VALUE 22
//冷暖值(可下发可上报)
//备注:
#define DPID_TEMP_VALUE 23
//彩光(可下发可上报)
//备注:类型：字符；
//Value: 000011112222；
//0000：H（色度：0-360，0X0000-0X0168）；
//1111：S (饱和：0-1000, 0X0000-0X03E8）；
//2222：V (明度：0-1000，0X0000-0X03E8)
#define DPID_COLOUR_DATA 24
//场景(可下发可上报)
//备注:类型：字符; 
//Value: 0011223344445555666677778888;
//00：情景号;
//11：单元切换间隔时间（0-100）;
//22：单元变化时间（0-100）;
//33：单元变化模式（0静态1跳变2渐变）;
//4444：H（色度：0-360，0X0000-0X0168）;
//5555：S (饱和：0-1000, 0X0000-0X03E8);
//6666：V (明度：0-1000，0X0000-0X03E8);
//7777：白光亮度（0-1000）;
//8888：色温值（0-1000）;
//注：数字1-8的标号对应有多少单元就有多少组
#define DPID_SCENE_DATA 25
//倒计时剩余时间(可下发可上报)
//备注:
#define DPID_COUNTDOWN 26
//音乐灯(只下发)
//备注:类型：字符串；
//Value: 011112222333344445555；
//0：   变化方式，0表示直接输出，1表示渐变；
//1111：H（色度：0-360，0X0000-0X0168）；
//2222：S (饱和：0-1000, 0X0000-0X03E8）；
//3333：V (明度：0-1000，0X0000-0X03E8）；
//4444：白光亮度（0-1000）；
//5555：色温值（0-1000）
#define DPID_MUSIC_DATA 27
//调节(只下发)
//备注:类型：字符串 ;
//Value: 011112222333344445555  ;
//0：   变化方式，0表示直接输出，1表示渐变;
//1111：H（色度：0-360，0X0000-0X0168）;
//2222：S (饱和：0-1000, 0X0000-0X03E8);
//3333：V (明度：0-1000，0X0000-0X03E8);
//4444：白光亮度（0-1000）;
//5555：色温值（0-1000）
#define DPID_CONTROL_DATA 28
//入睡(可下发可上报)
//备注:灯光按设定的时间淡出直至熄灭
#define DPID_SLEEP_MODE 31
//唤醒(可下发可上报)
//备注:灯光按设定的时间逐渐淡入直至设定的亮度
#define DPID_WAKEUP_MODE 32
//断电记忆(可下发可上报)
//备注:通电后，灯亮起的状态
#define DPID_POWER_MEMORY 33
//勿扰模式(可下发可上报)
//备注:适用经常停电区域，开启通电勿扰，通过APP关灯需连续两次上电才会亮灯
//Value：ABCCDDEEFFGG
//A：版本，初始版本0x00；
//B：模式，0x00初始默认值、0x01恢复记忆值、0x02用户定制；
//CC：色相 H，0~360；
//DD：饱和度 S，0~1000；
//EE：明度 V，0~1000；
//FF：亮度，0~1000；
//GG：色温，0~1000；
#define DPID_DO_NOT_DISTURB 34
//麦克风音乐律动(可下发可上报)
//备注:类型：  字符串
//Value：  AABBCCDDEEFFGGGGHHHHIIIIJJJJKKKKLLLLMMMMNNNN
//AA  版本
//BB  0-关闭，1-打开
//CC  模式编号，自定义从201开始
//DD  变换方式：0 - 呼吸模式，1 -跳变模式 ， 2 - 经典模式
//EE  变化速度
//FF  灵敏度
//GGGG  颜色1-色相饱和度
//HHHH  颜色2-色相饱和度
//......
//NNNN  颜色8-色相饱和度
#define DPID_MIC_MUSIC_DATA 42
//炫彩情景(可下发可上报)
//备注:专门用于幻彩灯带场景
//Value：ABCDEFGHIJJKLLM...
//A：版本号；
//B：情景模式编号；
//C：变化方式（0-静态、1-渐变、2跳变、3呼吸、4-闪烁、10-流水、11-彩虹）
//D：单元切换间隔时间（0-100）;
//E：单元变化时间（0-100）；
//FGH：设置项；
//I：亮度（亮度V：0~100）；
//JJ：颜色1（色度H：0-360）；
//K：饱和度1 (饱和度S：0-100)；
//LL：颜色2（色度H：0-360）；
//M：饱和度2（饱和度S：0~100）；
//注：有多少个颜色单元就有多少组，最多支持20组；
//每个字母代表一个字节
#define DPID_DREAMLIGHT_SCENE_MODE 51
//炫彩本地音乐律动(可下发可上报)
//备注:专门用于幻彩灯带本地音乐
//Value：ABCDEFGHIJKKLMMN...
//A：版本号；
//B：本地麦克风开关（0-关、1-开）；
//C：音乐模式编号；
//D：变化方式；
//E：变化速度（1-100）;
//F：灵敏度(1-100)；
//GHI：设置项；
//J：亮度（亮度V：0~100）；
//KK：颜色1（色度H：0-360）；
//L：饱和度1 (饱和度S：0-100)；
//MM：颜色2（色度H：0-360）；
//N：饱和度2（饱和度S：0~100）；
//注：有多少个颜色单元就有多少组，最多支持8组；
//每个字母代表一个字节
#define DPID_DREAMLIGHTMIC_MUSIC_DATA 52
//点数/长度设置(可下发可上报)
//备注:幻彩灯带裁剪之后重新设置长度
#define DPID_LIGHTPIXEL_NUMBER_SET 53

unsigned char dp_bool_value = 0;
long dp_value_value = 0;
unsigned char dp_enum_value = 0;
unsigned char dp_string_value[128] = {"Hi Tuya"};
uint16_t Hue=0; //HSV
uint8_t Sat=0;
uint8_t Val=0;
uint8_t scene_mode=0;
unsigned char hex[10] = {"0"};
unsigned char dp_array[][2] = {
  {DPID_SWITCH_LED, DP_TYPE_BOOL},
  {DPID_WORK_MODE, DP_TYPE_ENUM},
  {DPID_BRIGHT_VALUE, DP_TYPE_VALUE},
  {DPID_TEMP_VALUE, DP_TYPE_VALUE},
  {DPID_COLOUR_DATA, DP_TYPE_STRING},
  {DPID_SCENE_DATA, DP_TYPE_STRING},
  {DPID_COUNTDOWN, DP_TYPE_VALUE},
  {DPID_MUSIC_DATA, DP_TYPE_STRING},
  {DPID_CONTROL_DATA, DP_TYPE_STRING},
  {DPID_SLEEP_MODE, DP_TYPE_RAW},
  {DPID_WAKEUP_MODE, DP_TYPE_RAW},
  {DPID_POWER_MEMORY, DP_TYPE_RAW},
  {DPID_DO_NOT_DISTURB, DP_TYPE_BOOL},
  {DPID_MIC_MUSIC_DATA, DP_TYPE_STRING},
  {DPID_DREAMLIGHT_SCENE_MODE, DP_TYPE_RAW},
  {DPID_DREAMLIGHTMIC_MUSIC_DATA, DP_TYPE_RAW},
  {DPID_LIGHTPIXEL_NUMBER_SET, DP_TYPE_VALUE},
};


void setup() {
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'
  
  DebugSerial.begin(9600);
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(key_pin, INPUT_PULLUP);

  my_device.init(pid, mcu_ver);
  my_device.set_dp_cmd_total(dp_array, 17);
  my_device.dp_process_func_register(dp_process);
  my_device.dp_update_all_func_register(dp_update_all);

  last_time = millis();
  
}

void loop() 
{
  my_device.uart_service();

  //Enter the connection network mode when Pin7 is pressed.
  if (digitalRead(key_pin) == LOW) {
    delay(80);
    if (digitalRead(key_pin) == LOW) {
      my_device.mcu_set_wifi_mode(SMART_CONFIG);
    }
  }
  /* LED blinks when network is being connected */
  if ((my_device.mcu_get_wifi_work_state() != WIFI_LOW_POWER) && (my_device.mcu_get_wifi_work_state() != WIFI_CONN_CLOUD) && (my_device.mcu_get_wifi_work_state() != WIFI_SATE_UNKNOW)) {
    if (millis()- last_time >= 500) {
      last_time = millis();

      if (led_state == LOW) {
        led_state = HIGH;
      } else {
        led_state = LOW;
      }
      digitalWrite(LED_BUILTIN, led_state);
    }
  }

}

void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

void rainbow(int wait) {
  for(long firstPixelHue = 0; firstPixelHue < 3*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { 
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show();
    delay(wait);
  }
}

void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      for(int c=b; c<strip.numPixels(); c += 3) {
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}


 void colour_data_control( const unsigned char value[], u16 length)
 {
   u8 string_data[13];
    u16 h, s, v;
    u8 r, g, b;
    u16 hue;
    u8 sat,val;

    u32 c=0;
  
    string_data[0] = value[0]; //渐变、直接输出
    string_data[1] = value[1];
    string_data[2] = value[2];
    string_data[3] = value[3];
    string_data[4] = value[4];
    string_data[5] = value[5];
    string_data[6] = value[6];
    string_data[7] = value[7];
    string_data[8] = value[8];
    string_data[9] = value[9];
    string_data[10] = value[10];
    string_data[11] = value[11];
    string_data[12] = value[12];
    
  
    h = __str2short(__asc2hex(string_data[1]), __asc2hex(string_data[2]), __asc2hex(string_data[3]), __asc2hex(string_data[4]));
    s = __str2short(__asc2hex(string_data[5]), __asc2hex(string_data[6]), __asc2hex(string_data[7]), __asc2hex(string_data[8]));
    v = __str2short(__asc2hex(string_data[9]), __asc2hex(string_data[10]), __asc2hex(string_data[11]), __asc2hex(string_data[12]));

    

    // if (v <= 10) {
    //     v = 0;
    // } else {
    //     v = color_val_lmt_get(v);
    // }
    
    //hsv2rgb((float)h, (float)s / 1000.0, (float)v / 1000.0, &r , &g, &b);

    // c= r<<16|g<<8|b;
  hue=h*182;
  sat=s/4;
  val=v/4;
    c = strip.gamma32(strip.ColorHSV(hue,sat,val)); // hue -> RGB
    DebugSerial.println(hue);
    DebugSerial.println(sat);
    DebugSerial.println(val);
 
    
    strip.fill(c,0,PIXEL_COUNT);
    
    strip.show(); // Update strip with new contents

    //tuya_light_gamma_adjust(r, g, b, &mcu_default_color.red, &mcu_default_color.green, &mcu_default_color.blue);
  
    //printf("r=%d,g=%d,b=%d\r\n", mcu_default_color.red, mcu_default_color.green, mcu_default_color.blue);
    //rgb_init(mcu_default_color.red, mcu_default_color.green, mcu_default_color.blue);
 }
/**
 * @brief  str to short
 * @param[in] {a} Single Point
 * @param[in] {b} Single Point
 * @param[in] {c} Single Point
 * @param[in] {d} Single Point
 * @return Integrated value
 * @note   Null
 */
u32 __str2short(u32 a, u32 b, u32 c, u32 d)
{
    return (a << 12) | (b << 8) | (c << 4) | (d & 0xf);
}

/**
  * @brief ASCALL to Hex
  * @param[in] {asccode} 当前ASCALL值
  * @return Corresponding value
  * @retval None
  */
u8 __asc2hex(u8 asccode)
{
    u8 ret;
    
    if ('0' <= asccode && asccode <= '9')
        ret = asccode - '0';
    else if ('a' <= asccode && asccode <= 'f')
        ret = asccode - 'a' + 10;
    else if ('A' <= asccode && asccode <= 'F')
        ret = asccode - 'A' + 10;
    else
        ret = 0;
    
    return ret;
}

/**
  * @brief Normalized
  * @param[in] {dp_val} dp value
  * @return result
  * @retval None
  */
u16 color_val_lmt_get(u16 dp_val)
{
    u16 max = 255 * 100 / 100;
    u16 min = 255 * 1 / 100;
    
    return ((dp_val - 10) * (max - min) / (1000 - 10) + min);
}

/**
  * @brief hsv to rgb
  * @param[in] {h} tone
  * @param[in] {s} saturation
  * @param[in] {v} Lightness
  * @param[out] {color_r} red
  * @param[out] {color_g} green
  * @param[out] {color_b} blue
  * @retval None
  */
void hsv2rgb(float h, float s, float v, u8 *color_r, u8 *color_g, u8 *color_b)
{
    float h60, f;
    u32 h60f, hi;
  
    h60 = h / 60.0;
    h60f = h / 60;
  
    hi = ( signed int)h60f % 6;
    f = h60 - h60f;
  
    float p, q, t;
  
    p = v * (1 - s);
    q = v * (1 - f * s);
    t = v * (1 - (1 - f) * s);
  
    float r, g, b;
  
    r = g = b = 0;
    if (hi == 0) {
        r = v;          g = t;        b = p;
    } else if (hi == 1) {
        r = q;          g = v;        b = p;
    } else if (hi == 2) {
        r = p;          g = v;        b = t;
    } else if (hi == 3) {
        r = p;          g = q;        b = v;
    } else if (hi == 4) {
        r = t;          g = p;        b = v;
    } else if (hi == 5) {
        r = v;          g = p;        b = q;
    }
  
    DebugSerial.println(r);
    DebugSerial.println(g);
    DebugSerial.println(b);
    r = (r * (float)255);
    g = (g * (float)255);
    b = (b * (float)255);
  
    *color_r = r;
    *color_g = g;
    *color_b = b;
    
    // r *= 100;
    // g *= 100;
    // b *= 100;
  
    // *color_r = (r + 50) / 100;
    // *color_g = (g + 50) / 100;
    // *color_b = (b + 50) / 100;
}


unsigned char dp_process(unsigned char dpid,const unsigned char value[], unsigned short length)
{
  switch(dpid) {
    case DPID_SWITCH_LED:
      dp_bool_value = my_device.mcu_get_dp_download_data(dpid, value, length); /* Get the value of the down DP command */
      if (dp_bool_value) {
        //Turn on
        colorfill (strip.Color(10,10,10)); //上一次状态
      } else {
        //Turn off
        colorfill (strip.Color(0,0,0));
      }
      //Status changes should be reported.
      my_device.mcu_dp_update(dpid, value, length);
    break;
  
    case DPID_COUNTDOWN:  //倒计时
     colorfill (strip.Color( 255, 0,  0));
      my_device.mcu_dp_update(dpid, value, length);
    break;

    case DPID_MUSIC_DATA: //音乐律动  
        my_device.mcu_dp_update(dpid, value, length);
        colour_data_control(value, length);

    break;
    
     case DPID_DREAMLIGHT_SCENE_MODE: //炫彩情景
     my_device.mcu_dp_update(DPID_DREAMLIGHT_SCENE_MODE, value, length);
     scene_mode=value[1];
     switch(scene_mode){
       case 0:
          colorWipe(strip.Color(  20,   0,   20), 24); 
          break;
        case 1:
          colorWipe(strip.Color(0,0,20), 24);  
          break;
        case 2:
          colorWipe(strip.Color(20,0,0), 24); 
          break;
        case 3:
          colorWipe(strip.Color(20,0,40), 24); 
          break;
        case 4:
          theaterChase(strip.Color(20, 20, 20), 24);
          break;
        case 5:
          theaterChase(strip.Color(20,   0,   0), 24);
          break;
        case 6:
          theaterChase(strip.Color(  0,   0, 20), 24);
          break;
        case 7:
          rainbow(10);
          break;
        case 8:
          theaterChaseRainbow(50);
          break;
        case 9: //彩虹
          rainbow(50);  
          break;
          
     }
      break;

      case DPID_LIGHTPIXEL_NUMBER_SET: //长度设置
      my_device.mcu_dp_update(dpid, value, length);
      break;
      
    default:break;
  }
  return SUCCESS;
}

void dp_update_all(void)
{
  my_device.mcu_dp_update(DPID_SWITCH_LED, led_state, 1);
}

//拓展
void colorfill(uint32_t color) {
 strip.fill(color,0,PIXEL_COUNT);
    strip.show();                          //  Update strip to match   
  
}
