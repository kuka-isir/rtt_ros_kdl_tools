#ifndef __MQUEUE_CONNPOLICY__
#define __MQUEUE_CONNPOLICY__
#include <rtt/transports/mqueue/MQLib.hpp>
#include <rtt/ConnPolicy.hpp>
namespace RTT{
class MQConnPolicy
{
    public: static ConnPolicy mq_data(const std::string& name_id="", int data_size = 1024, int lock_policy = ConnPolicy::LOCK_FREE, bool init_connection = true, bool pull = false)
    {
        ConnPolicy cp = ConnPolicy::data(lock_policy,init_connection,pull);
        cp.transport = ORO_MQUEUE_PROTOCOL_ID;
        cp.name_id = name_id;
        cp.data_size = data_size;
        return cp;
    }
    public: static ConnPolicy mq_buffer(int size, const std::string& name_id="", int data_size = 1024, int lock_policy = ConnPolicy::LOCK_FREE, bool init_connection = false, bool pull = false)
    {
        ConnPolicy cp = ConnPolicy::buffer(size,lock_policy,init_connection,pull);
        cp.transport = ORO_MQUEUE_PROTOCOL_ID;
        cp.name_id = name_id;
        cp.data_size = data_size;
        return cp;
    }
};
}
#endif
