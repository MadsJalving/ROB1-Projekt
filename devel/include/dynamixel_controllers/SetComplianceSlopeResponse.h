// Generated by gencpp from file dynamixel_controllers/SetComplianceSlopeResponse.msg
// DO NOT EDIT!


#ifndef DYNAMIXEL_CONTROLLERS_MESSAGE_SETCOMPLIANCESLOPERESPONSE_H
#define DYNAMIXEL_CONTROLLERS_MESSAGE_SETCOMPLIANCESLOPERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dynamixel_controllers
{
template <class ContainerAllocator>
struct SetComplianceSlopeResponse_
{
  typedef SetComplianceSlopeResponse_<ContainerAllocator> Type;

  SetComplianceSlopeResponse_()
    {
    }
  SetComplianceSlopeResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator> const> ConstPtr;

}; // struct SetComplianceSlopeResponse_

typedef ::dynamixel_controllers::SetComplianceSlopeResponse_<std::allocator<void> > SetComplianceSlopeResponse;

typedef boost::shared_ptr< ::dynamixel_controllers::SetComplianceSlopeResponse > SetComplianceSlopeResponsePtr;
typedef boost::shared_ptr< ::dynamixel_controllers::SetComplianceSlopeResponse const> SetComplianceSlopeResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dynamixel_controllers

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': True, 'HasHeader': False}
// {}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dynamixel_controllers/SetComplianceSlopeResponse";
  }

  static const char* value(const ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetComplianceSlopeResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::dynamixel_controllers::SetComplianceSlopeResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // DYNAMIXEL_CONTROLLERS_MESSAGE_SETCOMPLIANCESLOPERESPONSE_H
