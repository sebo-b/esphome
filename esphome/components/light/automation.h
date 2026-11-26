#pragma once

#include "esphome/core/automation.h"
#include "light_state.h"
#include "addressable_light.h"

namespace esphome::light {

enum class LimitMode { CLAMP, DO_NOTHING };

template<bool HasTransitionLength, typename... Ts> class DimRelativeAction final : public Action<Ts...> {
 public:
  explicit DimRelativeAction(LightState *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(float, relative_brightness)

  template<typename V>
  void set_transition_length(V value)
    requires(HasTransitionLength)
  {
    this->transition_length_ = value;
  }

  void play(const Ts &...x) override {
    auto call = this->parent_->make_call();
    float rel = this->relative_brightness_.value(x...);
    float cur;
    this->parent_->remote_values.as_brightness(&cur);
    if ((limit_mode_ == LimitMode::DO_NOTHING) && ((cur < min_brightness_) || (cur > max_brightness_))) {
      return;
    }
    float new_brightness = clamp(cur + rel, min_brightness_, max_brightness_);
    call.set_state(new_brightness != 0.0f);
    call.set_brightness(new_brightness);

    if constexpr (HasTransitionLength) {
      call.set_transition_length(this->transition_length_.optional_value(x...));
    }
    call.perform();
  }

  void set_min_max_brightness(float min, float max) {
    this->min_brightness_ = min;
    this->max_brightness_ = max;
  }

  void set_limit_mode(LimitMode limit_mode) { this->limit_mode_ = limit_mode; }

 protected:
  LightState *parent_;
  float min_brightness_{0.0};
  float max_brightness_{1.0};
  LimitMode limit_mode_{LimitMode::CLAMP};
  struct NoTransition {};
  [[no_unique_address]] std::conditional_t<HasTransitionLength, TemplatableFn<uint32_t, Ts...>, NoTransition>
      transition_length_{};
};

// Cycle through the light's configured effects. `Forward` selects direction
// at compile time so the chosen branch is the only one that gets instantiated
// per action site. `include_none` is runtime so a single set of templates
// covers both the "wrap through None" and "skip None" variants.
template<bool Forward, typename... Ts> class LightEffectCycleAction final : public Action<Ts...> {
 public:
  explicit LightEffectCycleAction(LightState *parent) : parent_(parent) {}

  void set_include_none(bool include_none) { this->include_none_ = include_none; }

  void play(const Ts &...) override {
    size_t count = this->parent_->get_effect_count();
    if (count == 0) {
      return;
    }
    uint32_t current = this->parent_->get_current_effect_index();
    uint32_t next;
    if (this->include_none_) {
      uint32_t total = static_cast<uint32_t>(count) + 1;
      if constexpr (Forward) {
        next = (current + 1) % total;
      } else {
        next = (current + total - 1) % total;
      }
    } else {
      if constexpr (Forward) {
        next = (current % static_cast<uint32_t>(count)) + 1;
      } else {
        next = (current <= 1) ? static_cast<uint32_t>(count) : current - 1;
      }
    }
    auto call = this->parent_->turn_on();
    call.set_effect(next);
    call.perform();
  }

 protected:
  LightState *parent_;
  bool include_none_{false};
};

class LightTurnOnTrigger final : public Trigger<>, public LightRemoteValuesListener {
 public:
  explicit LightTurnOnTrigger(LightState *a_light) : light_(a_light) {
    a_light->add_remote_values_listener(this);
    this->last_on_ = a_light->current_values.is_on();
  }

  void on_light_remote_values_update() override {
    // using the remote value because of transitions we need to trigger as early as possible
    auto is_on = this->light_->remote_values.is_on();
    // only trigger when going from off to on
    auto should_trigger = is_on && !this->last_on_;
    // Set new state immediately so that trigger() doesn't devolve
    // into infinite loop
    this->last_on_ = is_on;
    if (should_trigger) {
      this->trigger();
    }
  }

 protected:
  LightState *light_;
  bool last_on_;
};

class LightTurnOffTrigger final : public Trigger<>, public LightTargetStateReachedListener {
 public:
  explicit LightTurnOffTrigger(LightState *a_light) : light_(a_light) {
    a_light->add_target_state_reached_listener(this);
  }

  void on_light_target_state_reached() override {
    auto is_on = this->light_->current_values.is_on();
    // only trigger when going from on to off
    if (!is_on) {
      this->trigger();
    }
  }

 protected:
  LightState *light_;
};

class LightStateTrigger final : public Trigger<>, public LightRemoteValuesListener {
 public:
  explicit LightStateTrigger(LightState *a_light) { a_light->add_remote_values_listener(this); }

  void on_light_remote_values_update() override { this->trigger(); }
};

// This is slightly ugly, but we can't log in headers, and can't make this a static method on AddressableSet
// due to the template. It's just a temporary warning anyway.
void addressableset_warn_about_scale(const char *field);

#ifdef USE_LIGHT_MOVE
/// light.move on brightness: toward max_brightness (up) or min_brightness (down) at a constant speed that crosses
/// 0-100% in full_range_ms; 0 or less changes at once. Up turns an off light on at min_brightness, and continues from
/// where a light that is fading out is. Down leaves an off light off. With a min_brightness of 0, the light stays on
/// until it gets there and then turns off, as Matter's MoveWithOnOff does. Like a transition, the call publishes its
/// target, the limit, as the state at once; light.stop publishes where the light got to. A move to 0% is not saved
/// when it starts, only when it ends or stops.
/// A move whose limit the light is at already stops a running move of brightness instead, so the light does not go on
/// the other way; a running move of color temperature is not affected.
/// A move that takes over a running transition holds the color temperature, or the cold and warm white channels, where
/// they are shown, so they do not move on or jump at the stop. RGB is not held: it keeps the target of the call.
/// Limits:
/// - An addressable light without an effect gets one color on all LEDs, the same end state as any turn_on or
///   dim_relative, without the per-pixel fade of its own transition.
/// - A light of the `homeassistant` platform shows nothing of a held move until the release: that platform drops writes
///   while a transition runs, and a move is one.
void move_brightness(LightState *state, bool up, int32_t full_range_ms, float min_brightness, float max_brightness);
/// light.move on color temperature: UP IS WARMER (more mireds), toward the light's max_mireds, and down is colder,
/// toward its min_mireds, at a constant speed that crosses the whole range in full_range_ms; 0 or less changes at
/// once. Only for a light that is on, in a mode with color temperature. It holds the brightness where it is shown, and
/// stops a running move of color temperature, not of brightness, when the light is at the limit already.
void move_color_temperature(LightState *state, bool up, int32_t full_range_ms);
#endif

template<typename... Ts> class AddressableSet final : public Action<Ts...> {
 public:
  explicit AddressableSet(LightState *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(int32_t, range_from)
  TEMPLATABLE_VALUE(int32_t, range_to)
  TEMPLATABLE_VALUE(float, color_brightness)
  TEMPLATABLE_VALUE(float, red)
  TEMPLATABLE_VALUE(float, green)
  TEMPLATABLE_VALUE(float, blue)
  TEMPLATABLE_VALUE(float, white)

  void play(const Ts &...x) override {
    auto *out = (AddressableLight *) this->parent_->get_output();
    const int32_t size = out->size();
    int32_t range_from = interpret_index(this->range_from_.value_or(x..., 0), size);
    if (range_from < 0 || range_from >= size)
      range_from = 0;

    int32_t range_to = interpret_index(this->range_to_.value_or(x..., size - 1) + 1, size);
    if (range_to < 0 || range_to >= size)
      range_to = size;

    uint8_t color_brightness =
        to_uint8_scale(this->color_brightness_.value_or(x..., this->parent_->remote_values.get_color_brightness()));
    auto range = out->range(range_from, range_to);
    if (this->red_.has_value())
      range.set_red(esp_scale8(to_uint8_compat(this->red_.value(x...), "red"), color_brightness));
    if (this->green_.has_value())
      range.set_green(esp_scale8(to_uint8_compat(this->green_.value(x...), "green"), color_brightness));
    if (this->blue_.has_value())
      range.set_blue(esp_scale8(to_uint8_compat(this->blue_.value(x...), "blue"), color_brightness));
    if (this->white_.has_value())
      range.set_white(to_uint8_compat(this->white_.value(x...), "white"));
    out->schedule_show();
  }

 protected:
  LightState *parent_;

  static inline int32_t interpret_index(int32_t index, int32_t size) { return index < 0 ? index + size : index; }

  // Historically, this action required uint8_t (0-255) for RGBW values from lambdas. Keep compatibility.
  static inline uint8_t to_uint8_compat(float value, const char *field) {
    if (value > 1.0f) {
      addressableset_warn_about_scale(field);
      return static_cast<uint8_t>(value);
    }
    return to_uint8_scale(value);
  }
};

}  // namespace esphome::light
