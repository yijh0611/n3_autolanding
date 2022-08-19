// Generated by gencpp from file dji_sdk/DroneTaskControlRequest.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_DRONETASKCONTROLREQUEST_H
#define DJI_SDK_MESSAGE_DRONETASKCONTROLREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dji_sdk
{
template <class ContainerAllocator>
struct DroneTaskControlRequest_
{
  typedef DroneTaskControlRequest_<ContainerAllocator> Type;

  DroneTaskControlRequest_()
    : task(0)  {
    }
  DroneTaskControlRequest_(const ContainerAllocator& _alloc)
    : task(0)  {
  (void)_alloc;
    }



   typedef uint8_t _task_type;
  _task_type task;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(TASK_GOHOME)
  #undef TASK_GOHOME
#endif
#if defined(_WIN32) && defined(TASK_TAKEOFF)
  #undef TASK_TAKEOFF
#endif
#if defined(_WIN32) && defined(TASK_LAND)
  #undef TASK_LAND
#endif

  enum {
    TASK_GOHOME = 1u,
    TASK_TAKEOFF = 4u,
    TASK_LAND = 6u,
  };


  typedef boost::shared_ptr< ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator> const> ConstPtr;

}; // struct DroneTaskControlRequest_

typedef ::dji_sdk::DroneTaskControlRequest_<std::allocator<void> > DroneTaskControlRequest;

typedef boost::shared_ptr< ::dji_sdk::DroneTaskControlRequest > DroneTaskControlRequestPtr;
typedef boost::shared_ptr< ::dji_sdk::DroneTaskControlRequest const> DroneTaskControlRequestConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator1> & lhs, const ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator2> & rhs)
{
  return lhs.task == rhs.task;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator1> & lhs, const ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dji_sdk

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a3d0779d88f1c88b74d6da1390832b1b";
  }

  static const char* value(const ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa3d0779d88f1c88bULL;
  static const uint64_t static_value2 = 0x74d6da1390832b1bULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/DroneTaskControlRequest";
  }

  static const char* value(const ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#constant for tasks\n"
"uint8 TASK_GOHOME = 1\n"
"uint8 TASK_TAKEOFF = 4\n"
"uint8 TASK_LAND = 6\n"
"\n"
"#request\n"
"uint8 task    # see constants above for possible tasks\n"
;
  }

  static const char* value(const ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.task);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DroneTaskControlRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::DroneTaskControlRequest_<ContainerAllocator>& v)
  {
    s << indent << "task: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.task);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_DRONETASKCONTROLREQUEST_H
