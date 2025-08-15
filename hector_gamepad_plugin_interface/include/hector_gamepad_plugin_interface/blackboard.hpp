#ifndef BLACKBOARD_HPP
#define BLACKBOARD_HPP

#include "yaml_helper.hpp" // For YAML::convert<T>::decode
#include <any>
#include <iostream>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <yaml-cpp/yaml.h>
namespace hector_gamepad_plugin_interface
{

/**
 * # Blackboard
 * A small typed key–value store backed by `std::any`.
 *
 * ## Keys & Values
 * - Keys are strings. API accepts `std::string_view` to avoid copies at the call-site,
 *   but (on C++17) lookups still construct a `std::string` internally.
 * - Values are stored in `std::any`. **C++17 requires stored types to be CopyConstructible.**
 *   - For move-only payloads (e.g., `std::unique_ptr<T>`), use a copyable handle (e.g., `std::shared_ptr<T>`).
 *  - Note: per C++17, std::any stores the decayed type (std::decay_t<T>):
 *    - const/volatile qualifiers and references are stripped on storage. (e.g.: stored const int → int)
 */
class Blackboard
{
public:
  Blackboard() = default;

  /**
   * Insert or overwrite a value under `key`.
   * Perfect-forwards the argument and emplaces the decayed type into the `std::any`.
   * @note Stored type must be CopyConstructible (C++17 `std::any` requirement).
   */
  template<class T>
  void set( const std::string_view key, T &&value )
  {
    auto &slot = data_[std::string( key )]; // materialize key (C++17 map API)
    slot.reset();
    slot.template emplace<std::remove_reference_t<T>>( std::forward<T>( value ) );
  }

  /**
   * Construct a value of type `T` in place under `key` and return a reference to it.
   * Any previous value is discarded.
   */
  template<class T, class... Args>
  T &emplace( std::string key, Args &&...args )
  {
    auto &slot = data_[std::move( key )];
    slot.reset();
    slot.template emplace<T>( std::forward<Args>( args )... );
    return *std::any_cast<T>( &slot ); // safe: just emplaced as T
  }

  /**
   * Return pointer to stored T if present and type matches; nullptr otherwise.
   * Pointer-form `std::any_cast` does not throw, but constructing the key string may.
   */
  template<class T>
  T *try_get( const std::string_view key )
  {
    const auto it = data_.find( std::string( key ) );
    if ( it == data_.end() )
      return nullptr;
    return std::any_cast<T>( &it->second );
  }

  /// Const overload of try_get.
  template<class T>
  const T *try_get( const std::string_view key ) const
  {
    const auto it = data_.find( std::string( key ) );
    if ( it == data_.end() )
      return nullptr;
    return std::any_cast<T>( &it->second );
  }

  /**
   * Get a reference to stored T or throw.
   * @throws std::out_of_range if key missing
   * @throws std::bad_any_cast if type mismatch
   */
  template<class T>
  T &at( const std::string_view key )
  {
    return any_ref<T>( data_.at( std::string( key ) ) );
  }

  /// Const overload of at(); throws on missing/mismatch.
  template<class T>
  const T &at( std::string_view key ) const
  {
    return any_cref<T>( data_.at( std::string( key ) ) );
  }

  /**
   * Return a copy of stored T if present and type matches; otherwise return `default_value`.
   * Only copies the result; no exceptions from `any_cast` pointer form.
   */
  template<class T>
  T value_or( const std::string_view key, T default_value ) const
  {
    if ( auto p = try_get<T>( key ) )
      return *p;
    return default_value;
  }

  /**
   * Return reference to stored T if present and type matches; otherwise emplace T(args...) and return it.
   */
  template<class T, class... Args>
  T &get_or_emplace( std::string key, Args &&...args )
  {
    if ( auto p = try_get<T>( key ) )
      return *p;
    return emplace<T>( std::move( key ), std::forward<Args>( args )... );
  }

  /// True if a key exists (regardless of stored type).
  bool contains( std::string_view key ) const
  {
    return data_.find( std::string( key ) ) != data_.end();
  }

  /// Erase by key if present.
  void erase( const std::string_view key )
  {
    const auto it = data_.find( std::string( key ) );
    if ( it != data_.end() )
      data_.erase( it );
  }

  /// Remove all entries.
  void clear() noexcept { data_.clear(); }

  /// Number of stored entries.
  std::size_t size() const noexcept { return data_.size(); }

  /// True if there are no entries.
  bool empty() const noexcept { return data_.empty(); }

  /**
   * Set multiple values from a YAML node recursively.
   * valid types of the items of the node are bool, int, double, string and list of these.
   * Recursively traverses the node and sets keys with their values.
   * - Scalar nodes are set as bool|int|double|string.
   * - Sequence nodes are set as lists of bool|int|double|string.
   * - Map nodes are traversed recursively, prefixing keys with the map's tag.
   * -> sequences of mixed types (e.g., [1, "a"]) will be stored as lists of strings.
   */

  bool set_from_yaml( const YAML::Node &node, const std::string &prefix )
  {
    if ( !node )
      return false;

    bool ok = true;

    if ( node.IsMap() ) {
      for ( const auto &kv : node ) {
        const auto key = kv.first.as<std::string>();
        const YAML::Node &val = kv.second;
        const std::string child = prefix.empty() ? key : ( prefix + "/" + key );

        if ( val.IsMap() ) {
          ok &= set_from_yaml( val, child );
        } else if ( val.IsSequence() ) {
          ok &= set_list_from_yaml( val, child );
        } else if ( val.IsScalar() ) {
          if ( auto b = is_bool( val ) ) {
            set( child, *b );
          } else if ( auto i = is_int( val ) ) {
            set( child, *i );
          } else if ( auto d = is_double( val ) ) {
            set( child, *d );
          } else if ( auto s = is_string( val ) ) {
            set( child, *s );
          } else {
            ok = false; // unknown scalar form
          }
        } else if ( val.IsNull() ) {
          // silently ignore nulls (policy choice)
        } else {
          ok = false;
        }
      }
      return ok;
    }

    if ( node.IsSequence() ) {
      // Store the whole sequence at 'prefix'
      return set_list_from_yaml( node, prefix );
    }

    if ( node.IsScalar() ) {
      if ( prefix.empty() )
        return false; // nowhere to put it
      if ( auto b = is_bool( node ) ) {
        set( prefix, *b );
      } else if ( auto i = is_int( node ) ) {
        set( prefix, *i );
      } else if ( auto d = is_double( node ) ) {
        set( prefix, *d );
      } else if ( auto s = is_string( node ) ) {
        set( prefix, *s );
      } else {
        return false;
      }
      return true;
    }

    if ( node.IsNull() ) {
      // ignore
      return true;
    }

    return false;
  }

  bool set_list_from_yaml( const YAML::Node &list, const std::string &key )
  {
    if ( !list.IsSequence() )
      return false;

    // Policy: empty sequence → store empty vector<string>
    if ( list.size() == 0 ) {
      set( key, std::vector<std::string>{} );
      return true;
    }

    // Reject nested maps/sequences and detect homogeneous scalar type.
    bool all_bool = true;
    bool all_int = true;
    bool all_double = true;
    bool all_string = true;

    for ( const auto &item : list ) {
      if ( item.IsMap() || item.IsSequence() ) {
        return false; // you said: maps in sequences are not allowed
      }
      if ( !is_bool( item ) )
        all_bool = false;
      if ( !is_int( item ) )
        all_int = false;
      if ( !is_double( item ) )
        all_double = false;
      if ( !is_string( item ) )
        all_string = false;
    }

    // Prefer int over double when both possible.
    if ( all_bool ) {
      std::vector<bool> v;
      v.reserve( list.size() );
      for ( const auto &item : list ) v.push_back( *is_bool( item ) );
      set( key, v );
      return true;
    }
    if ( all_int ) {
      std::vector<int64_t> v;
      v.reserve( list.size() );
      for ( const auto &item : list ) v.push_back( *is_int( item ) );
      set( key, v );
      return true;
    }
    if ( all_double ) {
      std::vector<double> v;
      v.reserve( list.size() );
      for ( const auto &item : list ) v.push_back( *is_double( item ) );
      set( key, v );
      return true;
    }
    if ( all_string ) {
      std::vector<std::string> v;
      v.reserve( list.size() );
      for ( const auto &item : list ) v.push_back( *is_string( item ) );
      set( key, v );
      return true;
    }

    // Mixed scalar types -> not allowed
    return false;
  }

private:
  std::unordered_map<std::string, std::any> data_;

  // Non-const any → T&
  template<class T>
  static T &any_ref( std::any &a )
  {
    using U = std::remove_reference_t<T>;
    if ( auto p = std::any_cast<U>( &a ) )
      return *p;
    throw std::bad_any_cast{};
  }

  // Const any → const T&
  template<class T>
  static const T &any_cref( const std::any &a )
  {
    using U = std::remove_reference_t<T>;
    if ( auto p = std::any_cast<U>( &a ) )
      return *p;
    throw std::bad_any_cast{};
  }
};

} // namespace hector_gamepad_plugin_interface
#endif // BLACKBOARD_HPP
