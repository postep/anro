// Generated by gencpp from file lab4/oint_control_srvRequest.msg
// DO NOT EDIT!


#ifndef LAB4_MESSAGE_OINT_CONTROL_SRVREQUEST_H
#define LAB4_MESSAGE_OINT_CONTROL_SRVREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace lab4
{
template <class ContainerAllocator>
struct oint_control_srvRequest_
{
  typedef oint_control_srvRequest_<ContainerAllocator> Type;

  oint_control_srvRequest_()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , time(0.0)
    , type()  {
    }
  oint_control_srvRequest_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , time(0.0)
    , type(_alloc)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;

   typedef double _time_type;
  _time_type time;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _type_type;
  _type_type type;




  typedef boost::shared_ptr< ::lab4::oint_control_srvRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lab4::oint_control_srvRequest_<ContainerAllocator> const> ConstPtr;

}; // struct oint_control_srvRequest_

typedef ::lab4::oint_control_srvRequest_<std::allocator<void> > oint_control_srvRequest;

typedef boost::shared_ptr< ::lab4::oint_control_srvRequest > oint_control_srvRequestPtr;
typedef boost::shared_ptr< ::lab4::oint_control_srvRequest const> oint_control_srvRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lab4::oint_control_srvRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lab4::oint_control_srvRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace lab4

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::lab4::oint_control_srvRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lab4::oint_control_srvRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lab4::oint_control_srvRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lab4::oint_control_srvRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lab4::oint_control_srvRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lab4::oint_control_srvRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lab4::oint_control_srvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "54f729053e32a7e8728d58944ecf89d2";
  }

  static const char* value(const ::lab4::oint_control_srvRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x54f729053e32a7e8ULL;
  static const uint64_t static_value2 = 0x728d58944ecf89d2ULL;
};

template<class ContainerAllocator>
struct DataType< ::lab4::oint_control_srvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lab4/oint_control_srvRequest";
  }

  static const char* value(const ::lab4::oint_control_srvRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lab4::oint_control_srvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n\
float64 y\n\
float64 z\n\
float64 time\n\
string type\n\
";
  }

  static const char* value(const ::lab4::oint_control_srvRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lab4::oint_control_srvRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.time);
      stream.next(m.type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct oint_control_srvRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lab4::oint_control_srvRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lab4::oint_control_srvRequest_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "time: ";
    Printer<double>::stream(s, indent + "  ", v.time);
    s << indent << "type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LAB4_MESSAGE_OINT_CONTROL_SRVREQUEST_H
