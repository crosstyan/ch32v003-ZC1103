// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: wrapper.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_wrapper_2eproto_2epb_2eh
#define GOOGLE_PROTOBUF_INCLUDED_wrapper_2eproto_2epb_2eh

#include <limits>
#include <string>
#include <type_traits>

#include "google/protobuf/port_def.inc"
#if PROTOBUF_VERSION < 4023000
#error "This file was generated by a newer version of protoc which is"
#error "incompatible with your Protocol Buffer headers. Please update"
#error "your headers."
#endif  // PROTOBUF_VERSION

#if 4023002 < PROTOBUF_MIN_PROTOC_VERSION
#error "This file was generated by an older version of protoc which is"
#error "incompatible with your Protocol Buffer headers. Please"
#error "regenerate this file with a newer version of protoc."
#endif  // PROTOBUF_MIN_PROTOC_VERSION
#include "google/protobuf/port_undef.inc"
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/arena.h"
#include "google/protobuf/arenastring.h"
#include "google/protobuf/generated_message_util.h"
#include "google/protobuf/metadata_lite.h"
#include "google/protobuf/generated_message_reflection.h"
#include "google/protobuf/message.h"
#include "google/protobuf/repeated_field.h"  // IWYU pragma: export
#include "google/protobuf/extension_set.h"  // IWYU pragma: export
#include "google/protobuf/unknown_field_set.h"
#include "spot_config.pb.h"
#include "spot.pb.h"
// @@protoc_insertion_point(includes)

// Must be included last.
#include "google/protobuf/port_def.inc"

#define PROTOBUF_INTERNAL_EXPORT_wrapper_2eproto

PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_wrapper_2eproto {
  static const ::uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable
    descriptor_table_wrapper_2eproto;
class Message;
struct MessageDefaultTypeInternal;
extern MessageDefaultTypeInternal _Message_default_instance_;
PROTOBUF_NAMESPACE_OPEN
template <>
::Message* Arena::CreateMaybeMessage<::Message>(Arena*);
PROTOBUF_NAMESPACE_CLOSE


// ===================================================================


// -------------------------------------------------------------------

class Message final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:Message) */ {
 public:
  inline Message() : Message(nullptr) {}
  ~Message() override;
  template<typename = void>
  explicit PROTOBUF_CONSTEXPR Message(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Message(const Message& from);
  Message(Message&& from) noexcept
    : Message() {
    *this = ::std::move(from);
  }

  inline Message& operator=(const Message& from) {
    CopyFrom(from);
    return *this;
  }
  inline Message& operator=(Message&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const Message& default_instance() {
    return *internal_default_instance();
  }
  enum DestinationCase {
    kDevice = 1,
    kBroadcast = 2,
    DESTINATION_NOT_SET = 0,
  };

  enum PayloadCase {
    kConfig = 3,
    kSpot = 4,
    PAYLOAD_NOT_SET = 0,
  };

  static inline const Message* internal_default_instance() {
    return reinterpret_cast<const Message*>(
               &_Message_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Message& a, Message& b) {
    a.Swap(&b);
  }
  inline void Swap(Message* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Message* other) {
    if (other == this) return;
    ABSL_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  Message* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<Message>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Message& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const Message& from) {
    Message::MergeImpl(*this, from);
  }
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  ::size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::uint8_t* _InternalSerialize(
      ::uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _impl_._cached_size_.Get(); }

  private:
  void SharedCtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Message* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::absl::string_view FullMessageName() {
    return "Message";
  }
  protected:
  explicit Message(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDeviceFieldNumber = 1,
    kBroadcastFieldNumber = 2,
    kConfigFieldNumber = 3,
    kSpotFieldNumber = 4,
  };
  // bytes device = 1;
  bool has_device() const;
  void clear_device() ;
  const std::string& device() const;




  template <typename Arg_ = const std::string&, typename... Args_>
  void set_device(Arg_&& arg, Args_... args);
  std::string* mutable_device();
  PROTOBUF_NODISCARD std::string* release_device();
  void set_allocated_device(std::string* ptr);

  private:
  const std::string& _internal_device() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_device(
      const std::string& value);
  std::string* _internal_mutable_device();

  public:
  // int32 broadcast = 2;
  bool has_broadcast() const;
  void clear_broadcast() ;
  ::int32_t broadcast() const;
  void set_broadcast(::int32_t value);

  private:
  ::int32_t _internal_broadcast() const;
  void _internal_set_broadcast(::int32_t value);

  public:
  // .SpotConfig config = 3;
  bool has_config() const;
  private:
  bool _internal_has_config() const;

  public:
  void clear_config() ;
  const ::SpotConfig& config() const;
  PROTOBUF_NODISCARD ::SpotConfig* release_config();
  ::SpotConfig* mutable_config();
  void set_allocated_config(::SpotConfig* config);
  private:
  const ::SpotConfig& _internal_config() const;
  ::SpotConfig* _internal_mutable_config();
  public:
  void unsafe_arena_set_allocated_config(
      ::SpotConfig* config);
  ::SpotConfig* unsafe_arena_release_config();
  // .Spot spot = 4;
  bool has_spot() const;
  private:
  bool _internal_has_spot() const;

  public:
  void clear_spot() ;
  const ::Spot& spot() const;
  PROTOBUF_NODISCARD ::Spot* release_spot();
  ::Spot* mutable_spot();
  void set_allocated_spot(::Spot* spot);
  private:
  const ::Spot& _internal_spot() const;
  ::Spot* _internal_mutable_spot();
  public:
  void unsafe_arena_set_allocated_spot(
      ::Spot* spot);
  ::Spot* unsafe_arena_release_spot();
  void clear_destination();
  DestinationCase destination_case() const;
  void clear_payload();
  PayloadCase payload_case() const;
  // @@protoc_insertion_point(class_scope:Message)
 private:
  class _Internal;
  void set_has_device();
  void set_has_broadcast();
  void set_has_config();
  void set_has_spot();

  inline bool has_destination() const;
  inline void clear_has_destination();

  inline bool has_payload() const;
  inline void clear_has_payload();

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    union DestinationUnion {
      constexpr DestinationUnion() : _constinit_{} {}
        ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized _constinit_;
      ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr device_;
      ::int32_t broadcast_;
    } destination_;
    union PayloadUnion {
      constexpr PayloadUnion() : _constinit_{} {}
        ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized _constinit_;
      ::SpotConfig* config_;
      ::Spot* spot_;
    } payload_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
    ::uint32_t _oneof_case_[2];

  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_wrapper_2eproto;
};

// ===================================================================




// ===================================================================


#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// -------------------------------------------------------------------

// Message

// bytes device = 1;
inline bool Message::has_device() const {
  return destination_case() == kDevice;
}
inline void Message::set_has_device() {
  _impl_._oneof_case_[0] = kDevice;
}
inline void Message::clear_device() {
  if (destination_case() == kDevice) {
    _impl_.destination_.device_.Destroy();
    clear_has_destination();
  }
}
inline const std::string& Message::device() const {
  // @@protoc_insertion_point(field_get:Message.device)
  return _internal_device();
}
template <typename Arg_, typename... Args_>
inline PROTOBUF_ALWAYS_INLINE void Message::set_device(Arg_&& arg,
                                                     Args_... args) {
  if (destination_case() != kDevice) {
    clear_destination();

    set_has_device();
    _impl_.destination_.device_.InitDefault();
  }
  _impl_.destination_.device_.SetBytes(static_cast<Arg_&&>(arg), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:Message.device)
}
inline std::string* Message::mutable_device() {
  std::string* _s = _internal_mutable_device();
  // @@protoc_insertion_point(field_mutable:Message.device)
  return _s;
}
inline const std::string& Message::_internal_device() const {
  if (destination_case() != kDevice) {
    return ::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited();
  }
  return _impl_.destination_.device_.Get();
}
inline void Message::_internal_set_device(const std::string& value) {
  if (destination_case() != kDevice) {
    clear_destination();

    set_has_device();
    _impl_.destination_.device_.InitDefault();
  }


  _impl_.destination_.device_.Set(value, GetArenaForAllocation());
}
inline std::string* Message::_internal_mutable_device() {
  if (destination_case() != kDevice) {
    clear_destination();

    set_has_device();
    _impl_.destination_.device_.InitDefault();
  }
  return _impl_.destination_.device_.Mutable( GetArenaForAllocation());
}
inline std::string* Message::release_device() {
  // @@protoc_insertion_point(field_release:Message.device)
  if (destination_case() != kDevice) {
    return nullptr;
  }
  clear_has_destination();
  return _impl_.destination_.device_.Release();
}
inline void Message::set_allocated_device(std::string* value) {
  if (has_destination()) {
    clear_destination();
  }
  if (value != nullptr) {
    set_has_device();
    _impl_.destination_.device_.InitAllocated(value, GetArenaForAllocation());
  }
  // @@protoc_insertion_point(field_set_allocated:Message.device)
}

// int32 broadcast = 2;
inline bool Message::has_broadcast() const {
  return destination_case() == kBroadcast;
}
inline void Message::set_has_broadcast() {
  _impl_._oneof_case_[0] = kBroadcast;
}
inline void Message::clear_broadcast() {
  if (destination_case() == kBroadcast) {
    _impl_.destination_.broadcast_ = 0;
    clear_has_destination();
  }
}
inline ::int32_t Message::broadcast() const {
  // @@protoc_insertion_point(field_get:Message.broadcast)
  return _internal_broadcast();
}
inline void Message::set_broadcast(::int32_t value) {
  _internal_set_broadcast(value);
  // @@protoc_insertion_point(field_set:Message.broadcast)
}
inline ::int32_t Message::_internal_broadcast() const {
  if (destination_case() == kBroadcast) {
    return _impl_.destination_.broadcast_;
  }
  return 0;
}
inline void Message::_internal_set_broadcast(::int32_t value) {
  if (destination_case() != kBroadcast) {
    clear_destination();
    set_has_broadcast();
  }
  _impl_.destination_.broadcast_ = value;
}

// .SpotConfig config = 3;
inline bool Message::has_config() const {
  return payload_case() == kConfig;
}
inline bool Message::_internal_has_config() const {
  return payload_case() == kConfig;
}
inline void Message::set_has_config() {
  _impl_._oneof_case_[1] = kConfig;
}
inline ::SpotConfig* Message::release_config() {
  // @@protoc_insertion_point(field_release:Message.config)
  if (payload_case() == kConfig) {
    clear_has_payload();
    ::SpotConfig* temp = _impl_.payload_.config_;
    if (GetArenaForAllocation() != nullptr) {
      temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
    }
    _impl_.payload_.config_ = nullptr;
    return temp;
  } else {
    return nullptr;
  }
}
inline const ::SpotConfig& Message::_internal_config() const {
  return payload_case() == kConfig
      ? *_impl_.payload_.config_
      : reinterpret_cast<::SpotConfig&>(::_SpotConfig_default_instance_);
}
inline const ::SpotConfig& Message::config() const {
  // @@protoc_insertion_point(field_get:Message.config)
  return _internal_config();
}
inline ::SpotConfig* Message::unsafe_arena_release_config() {
  // @@protoc_insertion_point(field_unsafe_arena_release:Message.config)
  if (payload_case() == kConfig) {
    clear_has_payload();
    ::SpotConfig* temp = _impl_.payload_.config_;
    _impl_.payload_.config_ = nullptr;
    return temp;
  } else {
    return nullptr;
  }
}
inline void Message::unsafe_arena_set_allocated_config(::SpotConfig* config) {
  clear_payload();
  if (config) {
    set_has_config();
    _impl_.payload_.config_ = config;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:Message.config)
}
inline ::SpotConfig* Message::_internal_mutable_config() {
  if (payload_case() != kConfig) {
    clear_payload();
    set_has_config();
    _impl_.payload_.config_ = CreateMaybeMessage< ::SpotConfig >(GetArenaForAllocation());
  }
  return _impl_.payload_.config_;
}
inline ::SpotConfig* Message::mutable_config() {
  ::SpotConfig* _msg = _internal_mutable_config();
  // @@protoc_insertion_point(field_mutable:Message.config)
  return _msg;
}

// .Spot spot = 4;
inline bool Message::has_spot() const {
  return payload_case() == kSpot;
}
inline bool Message::_internal_has_spot() const {
  return payload_case() == kSpot;
}
inline void Message::set_has_spot() {
  _impl_._oneof_case_[1] = kSpot;
}
inline ::Spot* Message::release_spot() {
  // @@protoc_insertion_point(field_release:Message.spot)
  if (payload_case() == kSpot) {
    clear_has_payload();
    ::Spot* temp = _impl_.payload_.spot_;
    if (GetArenaForAllocation() != nullptr) {
      temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
    }
    _impl_.payload_.spot_ = nullptr;
    return temp;
  } else {
    return nullptr;
  }
}
inline const ::Spot& Message::_internal_spot() const {
  return payload_case() == kSpot
      ? *_impl_.payload_.spot_
      : reinterpret_cast<::Spot&>(::_Spot_default_instance_);
}
inline const ::Spot& Message::spot() const {
  // @@protoc_insertion_point(field_get:Message.spot)
  return _internal_spot();
}
inline ::Spot* Message::unsafe_arena_release_spot() {
  // @@protoc_insertion_point(field_unsafe_arena_release:Message.spot)
  if (payload_case() == kSpot) {
    clear_has_payload();
    ::Spot* temp = _impl_.payload_.spot_;
    _impl_.payload_.spot_ = nullptr;
    return temp;
  } else {
    return nullptr;
  }
}
inline void Message::unsafe_arena_set_allocated_spot(::Spot* spot) {
  clear_payload();
  if (spot) {
    set_has_spot();
    _impl_.payload_.spot_ = spot;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:Message.spot)
}
inline ::Spot* Message::_internal_mutable_spot() {
  if (payload_case() != kSpot) {
    clear_payload();
    set_has_spot();
    _impl_.payload_.spot_ = CreateMaybeMessage< ::Spot >(GetArenaForAllocation());
  }
  return _impl_.payload_.spot_;
}
inline ::Spot* Message::mutable_spot() {
  ::Spot* _msg = _internal_mutable_spot();
  // @@protoc_insertion_point(field_mutable:Message.spot)
  return _msg;
}

inline bool Message::has_destination() const {
  return destination_case() != DESTINATION_NOT_SET;
}
inline void Message::clear_has_destination() {
  _impl_._oneof_case_[0] = DESTINATION_NOT_SET;
}
inline bool Message::has_payload() const {
  return payload_case() != PAYLOAD_NOT_SET;
}
inline void Message::clear_has_payload() {
  _impl_._oneof_case_[1] = PAYLOAD_NOT_SET;
}
inline Message::DestinationCase Message::destination_case() const {
  return Message::DestinationCase(_impl_._oneof_case_[0]);
}
inline Message::PayloadCase Message::payload_case() const {
  return Message::PayloadCase(_impl_._oneof_case_[1]);
}
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)


// @@protoc_insertion_point(global_scope)

#include "google/protobuf/port_undef.inc"

#endif  // GOOGLE_PROTOBUF_INCLUDED_wrapper_2eproto_2epb_2eh