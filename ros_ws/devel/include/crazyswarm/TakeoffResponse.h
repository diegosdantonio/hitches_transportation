// Generated by gencpp from file crazyswarm/TakeoffResponse.msg
// DO NOT EDIT!


#ifndef CRAZYSWARM_MESSAGE_TAKEOFFRESPONSE_H
#define CRAZYSWARM_MESSAGE_TAKEOFFRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace crazyswarm
{
template <class ContainerAllocator>
struct TakeoffResponse_
{
  typedef TakeoffResponse_<ContainerAllocator> Type;

  TakeoffResponse_()
    {
    }
  TakeoffResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::crazyswarm::TakeoffResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::crazyswarm::TakeoffResponse_<ContainerAllocator> const> ConstPtr;

}; // struct TakeoffResponse_

typedef ::crazyswarm::TakeoffResponse_<std::allocator<void> > TakeoffResponse;

typedef boost::shared_ptr< ::crazyswarm::TakeoffResponse > TakeoffResponsePtr;
typedef boost::shared_ptr< ::crazyswarm::TakeoffResponse const> TakeoffResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::crazyswarm::TakeoffResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::crazyswarm::TakeoffResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace crazyswarm

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::crazyswarm::TakeoffResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyswarm::TakeoffResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::crazyswarm::TakeoffResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::crazyswarm::TakeoffResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyswarm::TakeoffResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyswarm::TakeoffResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::crazyswarm::TakeoffResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::crazyswarm::TakeoffResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::crazyswarm::TakeoffResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "crazyswarm/TakeoffResponse";
  }

  static const char* value(const ::crazyswarm::TakeoffResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::crazyswarm::TakeoffResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::crazyswarm::TakeoffResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::crazyswarm::TakeoffResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TakeoffResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::crazyswarm::TakeoffResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::crazyswarm::TakeoffResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // CRAZYSWARM_MESSAGE_TAKEOFFRESPONSE_H