#ifndef BLACKBOARD_HPP
#define BLACKBOARD_HPP
#include <any>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <utility>

namespace hector_gamepad_plugin_interface
{
class Blackboard
{
public:
  // Insert/overwrite a value (perfect-forwarded).
  template <class T>
  void set(std::string_view key, T&& value)
  {
    // Note: std::any in C++17 requires the stored type to be CopyConstructible.
    data_[std::string(key)] = std::forward<T>(value);
  }

  // Emplace-construct a value in-place. Returns a reference to the stored value.
  template <class T, class... Args>
  T& emplace(std::string key, Args&&... args)
  {
    auto& slot = data_[std::move(key)];
    slot.reset();
    slot.template emplace<T>(std::forward<Args>(args)...);
    return *std::any_cast<T>(&slot);
  }

  // Get pointer to value if present and type matches; otherwise nullptr.
  template <class T>
  T* try_get(std::string_view key)
  {
    auto it = data_.find(std::string(key));
    if (it == data_.end())
      return nullptr;
    return std::any_cast<T>(&it->second);
  }

  template <class T>
  const T* try_get(std::string_view key) const
  {
    auto it = data_.find(std::string(key));
    if (it == data_.end())
      return nullptr;
    return std::any_cast<T>(&it->second);
  }

  // Get reference; throws std::out_of_range or std::bad_any_cast on mismatch.
  template <class T>
  T& at(std::string_view key)
  {
    return any_ref<T>(data_.at(std::string(key)));
  }
  template <class T>
  const T& at(std::string_view key) const
  {
    return any_cref<T>(data_.at(std::string(key)));
  }

  // Get a copy or a default if missing or wrong type.
  template <class T>
  T value_or(std::string_view key, T default_value) const
  {
    if (auto p = try_get<T>(key))
      return *p;
    return default_value;
  }

  // Get or emplace with constructor args. Returns reference to stored value.
  template <class T, class... Args>
  T& get_or_emplace(std::string key, Args&&... args)
  {
    if (auto p = try_get<T>(key))
      return *p;
    return emplace<T>(std::move(key), std::forward<Args>(args)...);
  }

  bool contains(std::string_view key) const
  {
    return data_.find(std::string(key)) != data_.end();
  }

  void erase(std::string_view key) { data_.erase(std::string(key)); }
  void clear() { data_.clear(); }

private:
  std::unordered_map<std::string, std::any> data_;

  // Non-const any → T&
  template <class T>
  static T& any_ref(std::any& a)
  {
    using U = std::remove_reference_t<T>;
    if (auto p = std::any_cast<U>(&a))
      return *p;
    throw std::bad_any_cast{};
  }

  // Const any → const T&
  template <class T>
  static const T& any_cref(const std::any& a)
  {
    using U = std::remove_reference_t<T>;
    if (auto p = std::any_cast<U>(&a))
      return *p;
    throw std::bad_any_cast{};
  }
};
}  // namespace hector_gamepad_plugin_interface
#endif  // BLACKBOARD_HPP
