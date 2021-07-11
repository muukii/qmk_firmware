#include QMK_KEYBOARD_H
#ifdef AUDIO_ENABLE
#include "muse.h"
#endif
#include "eeprom.h"
#include "keymap_german.h"
#include "keymap_nordic.h"
#include "keymap_french.h"
#include "keymap_spanish.h"
#include "keymap_hungarian.h"
#include "keymap_swedish.h"
#include "keymap_br_abnt2.h"
#include "keymap_canadian_multilingual.h"
#include "keymap_german_ch.h"
#include "keymap_jp.h"
#include "keymap_korean.h"
#include "keymap_bepo.h"
#include "keymap_italian.h"
#include "keymap_slovenian.h"
#include "keymap_lithuanian_azerty.h"
#include "keymap_danish.h"
#include "keymap_norwegian.h"
#include "keymap_portuguese.h"
#include "keymap_contributions.h"
#include "keymap_czech.h"
#include "keymap_romanian.h"
#include "keymap_russian.h"
#include "keymap_uk.h"
#include "keymap_estonian.h"
#include "keymap_belgian.h"
#include "keymap_us_international.h"

#define KC_MAC_UNDO LGUI(KC_Z)
#define KC_MAC_CUT LGUI(KC_X)
#define KC_MAC_COPY LGUI(KC_C)
#define KC_MAC_PASTE LGUI(KC_V)
#define KC_PC_UNDO LCTL(KC_Z)
#define KC_PC_CUT LCTL(KC_X)
#define KC_PC_COPY LCTL(KC_C)
#define KC_PC_PASTE LCTL(KC_V)
#define ES_LESS_MAC KC_GRAVE
#define ES_GRTR_MAC LSFT(KC_GRAVE)
#define ES_BSLS_MAC ALGR(KC_6)
#define NO_PIPE_ALT KC_GRAVE
#define NO_BSLS_ALT KC_EQUAL
#define LSA_T(kc) MT(MOD_LSFT | MOD_LALT, kc)
#define BP_NDSH_MAC ALGR(KC_8)

enum planck_keycodes {
  RGB_SLD = EZ_SAFE_RANGE,
  HSV_27_255_255,
  HSV_0_255_255,
  ST_MACRO_0,
  JP_LSPO,
  JP_RSPC,
};


enum planck_layers {
  _BASE,
  _LOWER,
  _RAISE,
  _ADJUST,
};

#define LOWER MO(_LOWER)
#define RAISE MO(_RAISE)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [_BASE] = LAYOUT_planck_grid(
                               KC_TAB,         KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,           KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           KC_BSPACE,
                               KC_LCTRL,       KC_A,           KC_S,           KC_D,           KC_F,           KC_G,           KC_H,           KC_J,           KC_K,           KC_L,           KC_SCOLON,      KC_QUOTE,
                               KC_LSHIFT,      KC_Z,           KC_X,           KC_C,           KC_V,           KC_B,           KC_N,           KC_M,           KC_COMMA,       KC_DOT,         KC_SLASH,       KC_ENTER,
                               KC_ESCAPE,      KC_TRANSPARENT, KC_LALT,        KC_LGUI,        LOWER,          KC_SPACE,       KC_NO,          RAISE,          KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT
                               ),

  [_LOWER] = LAYOUT_planck_grid(
                                KC_TILD,        KC_EXLM,        KC_AT,          KC_HASH,        KC_DLR,         KC_PERC,        KC_CIRC,        KC_AMPR,        KC_ASTR,        KC_LPRN,        KC_RPRN,        KC_NO,
                                KC_LCTRL,       LGUI(LSFT(KC_4)),LCTL(LGUI(LSFT(KC_4))),LGUI(LSFT(KC_5)),KC_ENTER,       KC_NO,          KC_NO,          KC_UNDS,        KC_PLUS,        KC_LCBR,        KC_RCBR,        KC_PIPE,
                                KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       KC_NO,          KC_BSPACE,      ST_MACRO_0,     KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,
                                JP_MEISU,       LCTL(LGUI(LSFT(KC_4))),KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_PGDOWN,      KC_PGUP,        KC_TRANSPARENT
                                ),

  [_RAISE] = LAYOUT_planck_grid(
                                KC_GRAVE,       KC_1,           KC_2,           KC_3,           KC_4,           KC_5,           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_DELETE,
                                KC_TRANSPARENT, KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,          KC_F6,          KC_MINUS,       KC_EQUAL,       KC_LBRACKET,    KC_RBRACKET,    KC_BSLASH,
                                KC_TRANSPARENT, KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,         KC_F12,         KC_NO,          KC_COMMA,       JP_DOT,         KC_SLASH,       KC_NO,
                                KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_NO,          KC_TRANSPARENT, KC_TRANSPARENT, KC_PGDOWN,      KC_PGUP,        JP_MKANA
                                ),

  [_ADJUST] = LAYOUT_planck_grid(
                                 KC_TRANSPARENT, TO(4),          TO(5),          KC_TRANSPARENT, KC_TRANSPARENT, RGB_TOG,        HSV_27_255_255, LED_LEVEL,      WEBUSB_PAIR,    HSV_0_255_255,  KC_TRANSPARENT, KC_TRANSPARENT,
                                 KC_TRANSPARENT, KC_TRANSPARENT, AU_ON,          AU_OFF,         AU_TOG,         KC_F14,         KC_F15,         TOGGLE_LAYER_COLOR,RGB_VAI,        RGB_VAD,        RGB_TOG,        RESET,
                                 KC_TRANSPARENT, KC_TRANSPARENT, MU_ON,          MU_OFF,         MU_TOG,         KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,RGB_MOD,        RGB_HUD,        RGB_HUD,        KC_TRANSPARENT, KC_TRANSPARENT,
                                 MU_TOG,         KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, TO(8),          KC_NO,          KC_TRANSPARENT, KC_MEDIA_PREV_TRACK,KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,KC_MEDIA_NEXT_TRACK
                                 ),
};

extern bool g_suspend_state;
extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][DRIVER_LED_TOTAL][3] = {
  [_BASE] = { {175, 241, 223},{180, 226, 196},{189, 205, 179},{202, 182, 168},{222, 159, 178},{238, 150, 204},{2, 141, 226},{14, 192, 237},{19, 235, 248},{16, 245, 255},{9, 231, 255},{1, 219, 255},{176, 237, 216},{182, 222, 193},{192, 199, 177},{207, 173, 168},{225, 158, 186},{243, 146, 213},{6, 153, 232},{16, 205, 239},{20, 246, 251},{14, 241, 255},{7, 228, 255},{255, 219, 255},{177, 236, 212},{184, 218, 191},{194, 194, 177},{210, 168, 170},{228, 155, 194},{246, 142, 218},{9, 165, 232},{17, 215, 240},{20, 254, 255},{12, 238, 255},{5, 226, 255},{253, 225, 255},{178, 232, 207},{186, 213, 190},{198, 188, 178},{215, 162, 176},{232, 152, 201},{251, 137, 225},{17, 223, 242},{18, 251, 255},{10, 236, 255},{3, 224, 255},{251, 229, 255} },

  [_LOWER] = { {175, 241, 223},{180, 226, 196},{189, 205, 179},{202, 182, 168},{222, 159, 178},{238, 150, 204},{2, 141, 226},{14, 192, 237},{19, 235, 248},{16, 245, 255},{9, 231, 255},{1, 219, 255},{176, 237, 216},{182, 222, 193},{192, 199, 177},{207, 173, 168},{225, 158, 186},{243, 146, 213},{6, 153, 232},{16, 205, 239},{20, 246, 251},{14, 241, 255},{7, 228, 255},{255, 219, 255},{177, 236, 212},{184, 218, 191},{194, 194, 177},{210, 168, 170},{228, 155, 194},{246, 142, 218},{9, 165, 232},{17, 215, 240},{20, 254, 255},{12, 238, 255},{5, 226, 255},{253, 225, 255},{178, 232, 207},{186, 213, 190},{198, 188, 178},{215, 162, 176},{232, 152, 201},{251, 137, 225},{17, 223, 242},{18, 251, 255},{10, 236, 255},{3, 224, 255},{251, 229, 255} },

  [_RAISE] = { {175, 241, 223},{180, 226, 196},{189, 205, 179},{202, 182, 168},{222, 159, 178},{238, 150, 204},{2, 141, 226},{14, 192, 237},{19, 235, 248},{16, 245, 255},{9, 231, 255},{1, 219, 255},{176, 237, 216},{182, 222, 193},{192, 199, 177},{207, 173, 168},{225, 158, 186},{243, 146, 213},{6, 153, 232},{16, 205, 239},{20, 246, 251},{14, 241, 255},{7, 228, 255},{255, 219, 255},{177, 236, 212},{184, 218, 191},{194, 194, 177},{210, 168, 170},{228, 155, 194},{246, 142, 218},{9, 165, 232},{17, 215, 240},{20, 254, 255},{12, 238, 255},{5, 226, 255},{253, 225, 255},{178, 232, 207},{186, 213, 190},{198, 188, 178},{215, 162, 176},{232, 152, 201},{251, 137, 225},{17, 223, 242},{18, 251, 255},{10, 236, 255},{3, 224, 255},{251, 229, 255} },

  [_ADJUST] = { {175, 241, 223},{180, 226, 196},{189, 205, 179},{202, 182, 168},{222, 159, 178},{238, 150, 204},{2, 141, 226},{14, 192, 237},{19, 235, 248},{16, 245, 255},{9, 231, 255},{1, 219, 255},{176, 237, 216},{182, 222, 193},{192, 199, 177},{207, 173, 168},{225, 158, 186},{243, 146, 213},{6, 153, 232},{16, 205, 239},{20, 246, 251},{14, 241, 255},{7, 228, 255},{255, 219, 255},{177, 236, 212},{184, 218, 191},{194, 194, 177},{210, 168, 170},{228, 155, 194},{246, 142, 218},{9, 165, 232},{17, 215, 240},{20, 254, 255},{12, 238, 255},{5, 226, 255},{253, 225, 255},{178, 232, 207},{186, 213, 190},{198, 188, 178},{215, 162, 176},{232, 152, 201},{251, 137, 225},{17, 223, 242},{18, 251, 255},{10, 236, 255},{3, 224, 255},{251, 229, 255} },
};

void set_layer_color(int layer) {
  for (int i = 0; i < DRIVER_LED_TOTAL; i++) {

//    const uint16_t key = keymaps_draft[layer][i];
//
//    if (
//        key == KC_NO
//        ) {
//      /// Set black color if the key is not available in the layer.
//      rgb_matrix_set_color( i, 0, 0, 0 );
//
//      continue;
//    }

    HSV hsv = {
        .h = pgm_read_byte(&ledmap[layer][i][0]),
        .s = pgm_read_byte(&ledmap[layer][i][1]),
        .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
      rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
      RGB rgb = hsv_to_rgb( hsv );
      float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
      rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }

  }
}

void rgb_matrix_indicators_user(void) {
  if (g_suspend_state || keyboard_config.disable_layer_led) { return; }
  switch (biton32(layer_state)) {
    case 0:
      set_layer_color(0);
      break;
    case 1:
      set_layer_color(1);
      break;
    case 2:
      set_layer_color(2);
      break;
    case 3:
      set_layer_color(3);
      break;
    case 4:
      set_layer_color(4);
      break;
    default:
      if (rgb_matrix_get_flags() == LED_FLAG_NONE)
        rgb_matrix_set_color_all(0, 0, 0);
      break;
  }
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case JP_LSPO:
      perform_space_cadet(record, keycode, KC_LSFT, KC_LSFT, KC_8);
      return false;
    case JP_RSPC:
      perform_space_cadet(record, keycode, KC_LSFT, KC_LSFT, KC_9);
      return false;
    case ST_MACRO_0:
      if (record->event.pressed) {
        SEND_STRING(SS_LCTL(SS_TAP(X_GRAVE)));

      }
      break;
    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
    case HSV_27_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(27,255,255);
      }
      return false;
    case HSV_0_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(0,255,255);
      }
      return false;
  }
  return true;
}

#ifdef AUDIO_ENABLE
bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

void encoder_update(bool clockwise) {
  if (muse_mode) {
    if (IS_LAYER_ON(_RAISE)) {
      if (clockwise) {
        muse_offset++;
      } else {
        muse_offset--;
      }
    } else {
      if (clockwise) {
        muse_tempo+=1;
      } else {
        muse_tempo-=1;
      }
    }
  } else {
    if (clockwise) {
#ifdef MOUSEKEY_ENABLE
      register_code(KC_MS_WH_DOWN);
      unregister_code(KC_MS_WH_DOWN);
#else
      register_code(KC_PGDN);
      unregister_code(KC_PGDN);
#endif
    } else {
#ifdef MOUSEKEY_ENABLE
      register_code(KC_MS_WH_UP);
      unregister_code(KC_MS_WH_UP);
#else
      register_code(KC_PGUP);
      unregister_code(KC_PGUP);
#endif
    }
  }
}

void matrix_scan_user(void) {
#ifdef AUDIO_ENABLE
  if (muse_mode) {
    if (muse_counter == 0) {
      uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
      if (muse_note != last_muse_note) {
        stop_note(compute_freq_for_midi_note(last_muse_note));
        play_note(compute_freq_for_midi_note(muse_note), 0xF);
        last_muse_note = muse_note;
      }
    }
    muse_counter = (muse_counter + 1) % muse_tempo;
  }
#endif
}

bool music_mask_user(uint16_t keycode) {
  switch (keycode) {
    case RAISE:
    case LOWER:
      return false;
    default:
      return true;
  }
}
#endif

uint32_t layer_state_set_user(uint32_t state) {
  return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}


