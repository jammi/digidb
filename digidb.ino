// This acts as an ADB host and a USB keyboard device
// allowing use of an ADB keyboard as if it were a
// USB keyboard.
//
// This uses a digispark (or appropriately configured
// attiny85).  A 470 ohm resistor should be placed between
// the ADB pin and 5V power.  The attiny85's internal
// pullup resistor can cause problems with ADB.
//
// ADB sends key up/down events, where USB sends the
// currently depressed keys.  This means we need to
// keep an internal buffer of the current state of the
// ADB keyboard.  Modifier keys are kept in the separate
// modifier bitmask and not added to the array of
// depressed keys.
// ADBUSBKeyDown() performs the appropriate modifier
// conversion if applicable, and adds the key to the buffer.
// ADBUSBKeyUp() will remove the key from the modifier
// bitmask or key buffer.
// Capslock is treated special here because on ADB keyboards
// the capslock key is mechanically latched, where on USB
// keyboards, the state of capslock is tracked by the USB
// host.  To match this, any state transition of the ADB
// capslock key is translated to both a key down and a key
// up event for USB.
//
// Keyboard LEDs are controlled by the USB host.  The USB
// set led commands are properly bridged to ADB Listen
// register 2 commands.
//
// Since the digispark uses the micronucleus bootloader,
// and this application software starts after a delay when
// that bootloader is present, it is not always obvious when
// the digispark is ready.  A startup sequence of toggling
// the keyboard's LEDs has been added to indicate the adapter
// is running and ready.
//
// Rob Braun <bbraun@synack.net> 2016

extern "C" {
  #include "usbdrv.h"
}
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

// Swaps Right Ctrl and Caps Lock around
// If you want to use it, alse modify the locking switch physically as described here:
// http://zacbir.net/blog/2014/12/19/getting-control-over-caps-lock/
#define CAPS_TO_CTRL
#ifdef CAPS_TO_CTRL
  #define CAPS_PHY 228
  #define CTRL_RIGHT_PHY 57
#else
  #define CAPS_PHY 57
  #define CTRL_RIGHT_PHY 228
#endif

// Pin # of ADB signal
#define ADB 2
#define ADB_PORT        PORTB
#define ADB_PIN         PINB
#define ADB_DDR         DDRB
#define ADB_DATA_BIT    ADB
#define LED 1

// For identifying reports
#define KeyboardID 0x01
#define MouseID 0x02

// For selecting which report to send
#define ADBUSB_KEYBOARD 1
#define ADBUSB_MOUSE    2

#define data_lo() (ADB_DDR |=  (1<<ADB_DATA_BIT))
#define data_hi() (ADB_DDR &= ~(1<<ADB_DATA_BIT))
#define data_in() (ADB_PIN & (1<<ADB_DATA_BIT))

PROGMEM static const uint8_t adb2usb[] {
  // auto generated
  4, // a 0x00
  22, // s 0x01
  7, // d 0x02
  9, // f 0x03
  11, // h 0x04
  10, // g 0x05
  29, // z 0x06
  27, // x 0x07
  6, // c 0x08
  25, // v 0x09
  100, // § section sign on intl 0x0a
      // (physically where grave is on US)
  5, // b 0x0b
  20, // q 0x0c
  26, // w 0x0d
  8, // e 0x0e
  21, // r 0x0f
  28, // y 0x10
  23, // t 0x11
  30, // 1 0x12
  31, // 2 0x13
  32, // 3 0x14
  33, // 4 0x15
  35, // 6 0x16
  34, // 5 0x17
  46, // = 0x18
  38, // 9 0x19
  36, // 7 0x1a
  45, // - 0x1b
  37, // 8 0x1c
  39, // 0 0x1d
  48, // ] 0x1e
  18, // o 0x1f
  24, // u 0x20
  47, // [ 0x21
  12, // i 0x22
  19, // p 0x23
  40, // enter 0x24
  15, // l 0x25
  13, // j 0x26
  52, // ' 0x27
  14, // k 0x28
  51, // ; 0x29
  49, // \ 0x2a
  54, // , 0x2b
  56, // / 0x2c
  17, // n 0x2d
  16, // m 0x2e
  55, // . 0x2f
  43, // tab 0x30
  44, // space 0x31
  53, // ` grave 0x32
  42, // backspace 0x33
  0xff, // 0x34
  41, // esc 0x35
  224, // left control 0x36
  227, // left cmd 0x37
  225, // left shift 0x38
  CAPS_PHY, // capslock/right ctrl 0x39
  226, // left opt 0x3a
  80, // left 0x3b
  79, // right 0x3c
  81, // down 0x3d
  82, // up 0x3e
  0xff, // 0x3f
  0xff, // 0x40
  99, // kp 0x41
  0xff, // 0x42
  85, // kp 0x43
  0xff, // 0x44
  87, // kp 0x45
  0xff, // 0x46
  156, // clear 0x47
  0xff, // 0x48
  0xff, // 0x49
  0xff, // 0x4a
  84, // kp 0x4b
  88, // kp 0x4c
  0xff, // 0x4d
  86, // kp 0x4e
  0xff, // 0x4f
  0xff, // 0x50
  103, // kp 0x51
  98, // kp 0x52
  89, // kp 0x53
  90, // kp 0x54
  91, // kp 0x55
  92, // kp 0x56
  93, // kp 0x57
  94, // kp 0x58
  95, // kp 0x59
  0xff, // 0x5a
  96, // kp 0x5b
  97, // kp 0x5c
  0xff, // 0x5d
  0xff, // 0x5e
  0xff, // 0x5f
  62, // f5 0x60
  63, // f6 0x61
  64, // f7 0x62
  60, // f3 0x63
  65, // f8 0x64
  66, // f9 0x65
  0xff, // 0x66
  68, // f11 0x67
  0xff, // 0x68
  104, // f13 0x69
  0xff, // 0x6a
  105, // f14 0x6b
  0xff, // 0x6c
  67, // f10 0x6d
  0xff, // 0x6e
  69, // f12 0x6f
  0xff, // 0x70
  106, // f15 0x71
  117, // help 0x72
  74, // home 0x73
  75, // pg 0x74
  76, // del 0x75
  61, // f4 0x76
  77, // end 0x77
  59, // f2 0x78
  78, // pg 0x79
  58, // f1 0x7a
  229, // right shift 0x7b
  230, // right alt/opt 0x7c
  CTRL_RIGHT_PHY, // right control 0x7d
};

#define ADB2USB_LAST 0x7d

// Maximum number of keys we can have depressed (minus modifiers)
// low speed devices can only send 8 bytes, so if we have
// report id for composite, we have to restrict this to 5 or less
#define MAX_KEYS 5

PROGMEM const char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = { /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x85, KeyboardID,              //   REPORT_ID (KeyboardID)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)(Key Codes)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)(224)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)(231)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs) ; Modifier byte
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs) ; Reserved byte
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs) ; LED report
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs) ; LED report padding
    0x95, MAX_KEYS,                //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)(Key Codes)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))(0)
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)(101)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0,                          // END_COLLECTION
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                    // USAGE (Mouse)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x85, MouseID,                 //   REPORT_ID (MouseID)
    0x05, 0x09,                    //   USAGE_PAGE (Button)
    0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
    0x29, 0x08,                    //   USAGE_MAXIMUM (Button 8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Ary,Abs)
    0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    0xc0,                          //   END_COLLECTION
    0xc0                           // END_COLLECTION
};

#define MOD_CONTROL_LEFT    (1<<0)
#define MOD_SHIFT_LEFT      (1<<1)
#define MOD_ALT_LEFT        (1<<2)
#define MOD_GUI_LEFT        (1<<3)
#define MOD_CONTROL_RIGHT   (1<<4)
#define MOD_SHIFT_RIGHT     (1<<5)
#define MOD_ALT_RIGHT       (1<<6)
#define MOD_GUI_RIGHT       (1<<7)

typedef struct  {
  uint8_t reportID;
  uint8_t modifiers;
  uint8_t reserved;
  uint8_t keys[MAX_KEYS];
} adbusb_report_t;

typedef struct {
  uint8_t reportID;
  uint8_t buttons;
  int8_t x;
  int8_t y;
} mouse_report_t;

adbusb_report_t adbusb_report;
mouse_report_t mouse_report;
static uint8_t idle_sav;
uint8_t adbleds = 0x07;
uint8_t srq = 0;
uint8_t curaddr;

void ADBUSBSetup(void) {
  cli();
  usbDeviceDisconnect();
  _delay_ms(250);
  usbDeviceConnect();
  usbInit();
  sei();
  memset(&adbusb_report, 0, sizeof(adbusb_report));
  memset(&mouse_report, 0, sizeof(mouse_report));
  adbusb_report.reportID = KeyboardID;
  mouse_report.reportID = MouseID;
}

// This sends a report
void ADBUSBSend(uint8_t which) {
  while (!usbInterruptIsReady()) {
    // Note: We wait until we can send keystroke
    //       so we know the previous keystroke was
    //       sent.
    usbPoll();
    _delay_ms(1);
  }
  if (which == ADBUSB_KEYBOARD) {
    adbusb_report.reportID = KeyboardID;
    usbSetInterrupt((unsigned char*)&adbusb_report, sizeof(adbusb_report));
  }
  else if (which == ADBUSB_MOUSE) {
    mouse_report.reportID = MouseID;
    usbSetInterrupt((unsigned char*)&mouse_report, sizeof(mouse_report));
  }
  while (!usbInterruptIsReady()) {
    // Note: We wait until we can send keystroke
    //       so we know the previous keystroke was
    //       sent.
    usbPoll();
    _delay_ms(1);
  }
}

// Maps modifier key codes to the USB modifiers bitmask value
uint8_t ADBUSBToModifier(uint8_t key) {
  switch(key) {
    case 224: return MOD_CONTROL_LEFT;
    case 225: return MOD_SHIFT_LEFT;
    case 226: return MOD_ALT_LEFT;
    case 227: return MOD_GUI_LEFT;
    case 228: return MOD_CONTROL_RIGHT;
    case 229: return MOD_SHIFT_RIGHT;
    case 230: return MOD_ALT_RIGHT;
    case 231: return MOD_GUI_RIGHT;
  }
  return 0;
}

void ADBUSBKeyDown(uint8_t key) {
  uint8_t i;
  i = ADBUSBToModifier(key);
  if (i) {
    adbusb_report.modifiers |= i;
    return;
  }

  // Check if the key is already in the report
  for(i = 0; i < MAX_KEYS; i++) {
    // If it is, nothing to do here
    if (adbusb_report.keys[i] == key) {
      return;
    }

    // The array is front loaded, so if we find an
    // empty spot, we're at the end and should use it.
    // If we never find an empty spot, there's too
    // many keys down, so drop it.
    if (adbusb_report.keys[i] == 0) {
      adbusb_report.keys[i] = key;
      return;
    }
  }

  return;
}

void ADBUSBKeyUp(uint8_t key) {
  uint8_t i;
  i = ADBUSBToModifier(key);
  if (i) {
    adbusb_report.modifiers &= ~i;
    return;
  }

  // remove key from report
  for(i = 0; i < MAX_KEYS; i++) {
    // found the key, keep the array front loaded
    if(adbusb_report.keys[i] == key) {
      for(; i < MAX_KEYS-1; i++) {
        adbusb_report.keys[i] = adbusb_report.keys[i+1];
      }
      adbusb_report.keys[MAX_KEYS-1] = 0;
      return;
    }
  }
}

#define KEY_1 30
#define KEY_0 39
#define KEY_CAPS 57
#define KEY_UP 82
#define KEY_DOWN 81
#define KEY_LEFT 80
#define KEY_RIGHT 79

#ifdef DEBUG
// This will type out the bits in the argument as if
// the keys 0 and 1 were pressed.  Useful for debugging.
void ADBUSBTypeBits(uint8_t c) {
  for(int8_t i = 7; i >= 0; i--) {
    uint8_t key = KEY_0;
    if(c & (1<<i)) key = KEY_1;
    ADBUSBKeyDown(key);
    ADBUSBSend(ADBUSB_KEYBOARD);
    ADBUSBKeyUp(key);
    ADBUSBSend(ADBUSB_KEYBOARD);
  }
  ADBUSBKeyDown(40);
  ADBUSBSend(ADBUSB_KEYBOARD);
  ADBUSBKeyUp(40);
  ADBUSBSend(ADBUSB_KEYBOARD);
}
#endif

static inline void ADBUSBKeyUpDown(uint16_t state, uint8_t key) {
    if (key == 0xff) {
      return;
    }
    if (state) {
#ifndef CAPS_TO_CTRL
      if(key == KEY_CAPS) {
        ADBUSBKeyDown(KEY_CAPS);
      } else {
        ADBUSBKeyUp(key);
      }
#endif
      ADBUSBKeyUp(key);
    }
    else {
      ADBUSBKeyDown(key);
    }
}

void ADBUSBDelay(long milli) {
  unsigned long last = millis();
  while (milli > 0) {
    unsigned long now = millis();
    milli -= now - last;
    last = now;
    usbPoll();
  }
}

uchar usbFunctionSetup(uchar data[8]) {
  usbRequest_t    *rq = (usbRequest_t *)((void *)data);

  if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {
    /* class request type */
    if (rq->bRequest == USBRQ_HID_GET_REPORT) {
      /* wValue: ReportType (highbyte), ReportID (lowbyte) */

      /* we only have one report type, so don't look at wValue */
      usbMsgPtr = (unsigned char *)&adbusb_report;
      adbusb_report.reportID = KeyboardID;
      adbusb_report.modifiers = 0;
      adbusb_report.keys[0] = 0;
      return sizeof(adbusb_report);
    }
    else if (rq->bRequest == USBRQ_HID_SET_REPORT) {
      // word == 1 if we're not using report id's
      // and == 2 if we are (first byte is report id)
      return (rq->wLength.word == 2) ? USB_NO_MSG : 0;
    }
    else if (rq->bRequest == USBRQ_HID_GET_IDLE) {
      usbMsgPtr = &idle_sav;
      return 1;
    }
    else if (rq->bRequest == USBRQ_HID_SET_IDLE) {
      idle_sav = rq->wValue.bytes[1];
    }
  }
  return 0;
}

uchar usbFunctionWrite(uchar * data, uchar len) {
  // ADB and USB LED bitmasks line up exactly, although
  // the states are inverted.
  // [1] if we're using report id's, otherwise [0]
  adbleds = ~(data[1] & 0x07);
  adbleds |= 0x80;
  return 1;
}

uint8_t MapADBToUSB(uint8_t key) {
  if (key <= ADB2USB_LAST) {
    return pgm_read_byte_near(adb2usb+key);
  }
  return 0xFF;
}

#if MAPMODIFIERS
uint8_t MapADBModifierToUSB(uint16_t adbmodifier) {
  uint8_t usbmodifier = 0;
  if (!(adbmodifier & (1<<8)) ) {
    usbmodifier |= MOD_GUI_LEFT;
  }
  if (!(adbmodifier & (1<<9)) ) {
    usbmodifier |= MOD_ALT_LEFT;
  }
  if (!(adbmodifier & (1<<10)) ) {
    usbmodifier |= MOD_SHIFT_LEFT;
  }
  if (!(adbmodifier & (1<<11)) ) {
    usbmodifier |= MOD_CONTROL_LEFT;
  }
  // unmapped: reset, caps, delete
  return usbmodifier;
}
#endif

void ADBSendKey(uint16_t adbkeys) {
#ifdef DEBUG
  ADBUSBTypeBits((adbkeys>>8) & 0x7F);
  ADBUSBTypeBits(adbkeys & 0x7F);
#endif
  uint8_t usbkey = MapADBToUSB((adbkeys>>8) & 0x7F);

  ADBUSBKeyUpDown((adbkeys & (1<<15)), usbkey);

  usbkey = MapADBToUSB(adbkeys & 0x7F);
  ADBUSBKeyUpDown((adbkeys & (1<<7)), usbkey);

  ADBUSBSend(ADBUSB_KEYBOARD);

#ifndef CAPS_TO_CTRL
  // With ADB keyboards, the capslock key is mechanically
  // locked in place, so the state is physical.
  // With USB keyboards, the capslock state is managed
  // by the USB host.  To keep the two in sync, any
  // state change for an ADB capslock key results in
  // a key up AND key down USB event.
  for(uint8_t i = 0; i < MAX_KEYS; i++) {
    if(adbusb_report.keys[i] == KEY_CAPS) {
      ADBUSBKeyUp(KEY_CAPS);
      ADBUSBSend(ADBUSB_KEYBOARD);
      break;
    }
  }
#endif
}

void ADBSendMouse(uint16_t data) {
  if (data & 0x8000) {
    mouse_report.buttons = 0;
  }
  else {
    mouse_report.buttons = 1;
  }
  //mouse_report.y = (data >> 7) & 0xFE;
  //mouse_report.x = ((data & 0xFF) << 1) & 0xFE;
  mouse_report.y = (data >> 8) & 0x7F;
  if (data & (1<<14)) {
    mouse_report.y |= 0x80;
  }
  mouse_report.x = data & 0x7F;
  if (data & (1<<6)) {
    mouse_report.x |= 0x80;
  }
  ADBUSBSend(ADBUSB_MOUSE);
}

static inline uint16_t wait_data_lo(uint16_t us) {
  do {
    if (!data_in()) {
      break;
    }
    _delay_us(1 - (6 * 1000000.0 / F_CPU));
  }
  while (--us);
  return us;
}

static inline uint16_t wait_data_hi(uint16_t us) {
  do {
    if (data_in()) {
      break;
    }
    _delay_us(1 - (6 * 1000000.0 / F_CPU));
  }
  while (--us);
  return us;
}

static inline void place_bit0(void) {
  data_lo();
  _delay_us(65);
  data_hi();
  _delay_us(35);
}

static inline void place_bit1(void) {
  data_lo();
  _delay_us(35);
  data_hi();
  _delay_us(65);
}

static inline void attnsync(void) {
  data_lo();
  _delay_us(800);
  data_hi();
  _delay_us(65);
}

static inline void send_byte(uint8_t data) {
  for (int i = 0; i < 8; i++) {
    if (data&(0x80>>i)) {
      place_bit1();
    }
    else {
      place_bit0();
    }
  }
}

static uint16_t inline recvword(uint16_t initial_wait) {
  uint8_t bits;
  uint16_t data = 0;
  //digitalWrite(LED, HIGH);
  uint16_t waittime = wait_data_lo(initial_wait);
  if(!waittime) {
    goto out;
  }
  waittime = wait_data_hi(initial_wait);
  if(!waittime) {
    goto out;
  }
  waittime = wait_data_lo(initial_wait);
  if(!waittime) {
    goto out;
  }
  cli();
  for(bits = 0; bits < 16; bits++) {
    uint8_t lo = wait_data_hi(130);
    if(!lo) {
      goto out;
    }

    uint8_t hi = wait_data_lo(lo);
    if(!hi) {
      goto out;
    }
    hi = lo - hi;
    lo = 130 - lo;
    data <<= 1;
    if(lo < hi) {
      data |= 1;
    }
  }
  sei();
  wait_data_hi(400);
  wait_data_lo(100);
  waittime = wait_data_hi(400);
  if (waittime > 0 && waittime < 300) {
    srq = 1;
  }
  return data;
out:
  sei();
  return 0xFFFF;
}

uint16_t ADBSendCommand(uint8_t cmd) {
  _delay_us(10);
  attnsync();
  send_byte(cmd); // talk addr 2, reg 0 mouse
  place_bit0(); // stop bit
  pinMode(ADB, INPUT);
  uint16_t waittime = wait_data_hi(300);
  if (waittime && waittime < 260) {
    srq = 1;
  }
  _delay_us(20);
  return recvword(500);
}

// To enable the ability to differentiate between
// left and right modifier keys, we need to set
// the device handler id in register 3 to 3.
void enable_extended(void) {
  attnsync();
  send_byte(0x2B); // addr 2, listen register 3
  place_bit0(); // stop bit
  _delay_us(100);
  //cli();
  place_bit1();
  send_byte(0x62); // default values, reserved bits 0, enable srq, and 1 if not used
  send_byte(0x03); // set device handler id to 3 to enable extended keyboard
  place_bit0();
  //sei();
}

void set_leds(uint8_t leds) {
  attnsync();
  send_byte(0x2A);
  place_bit0();
  _delay_us(100);
  //cli();
  place_bit1();
  send_byte(0x00);
  send_byte(leds);
  place_bit0();
  //sei();
}

void setup() {
  // put your setup code here, to run once:
  wdt_enable(WDTO_8S);
  wdt_reset();
  ADBUSBSetup();
  wdt_reset();
  srq = 0;
  curaddr = 0x30;
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(ADB, INPUT);
  // Use the keyboard LEDs
  // to indicate when we're up and running
  _delay_us(500);
  usbPoll();
  enable_extended();
  usbPoll();
  _delay_us(100);
  set_leds(0x03);
  ADBUSBDelay(500);
  set_leds(0x05);
  ADBUSBDelay(500);
  set_leds(0x06);
  ADBUSBDelay(500);
  set_leds(0x07);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t keys;
  wdt_reset();
  usbPoll();
  // See if we need to update LED state
  if (adbleds & 0x80) {
    adbleds &= ~0x80;
    set_leds(adbleds);
  }
  usbPoll();
  if (srq) {
    curaddr ^= 0x10; // toggle address between 0x20 and 0x30
    srq = 0;
  }
  keys = ADBSendCommand(curaddr | 0x0C);
  if (keys != 0xFFFF) {
    if (curaddr == 0x30) ADBSendMouse(keys);
    else if (curaddr == 0x20) ADBSendKey(keys);
  }
}
