#pragma once

#ifdef AUDIO_ENABLE
#define STARTUP_SONG SONG(PLANCK_SOUND)
#endif

#define MIDI_BASIC

#define ENCODER_RESOLUTION 4

/*
  Set any config.h overrides for your specific keymap here.
  See config.h options at https://docs.qmk.fm/#/config_options?id=the-configh-file
*/
#define ORYX_CONFIGURATOR
#undef RGBLIGHT_HUE_STEP
#define RGBLIGHT_HUE_STEP 20

#undef RGBLIGHT_BRI_STEP
#define RGBLIGHT_BRI_STEP 20

#define FIRMWARE_VERSION u8"VqDX5/yZy90"
#undef RGB_DISABLE_WHEN_USB_SUSPENDED
