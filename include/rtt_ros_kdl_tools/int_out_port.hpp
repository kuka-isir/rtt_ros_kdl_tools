#ifndef __IN_OUT_PORT_HPP__
#define __IN_OUT_PORT_HPP__

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/InputPort.hpp>

template <typename T>
class has_resize
{
    typedef char one;
    typedef long two;

    template <typename C> static one test( typeof(&C::resize) ) ;
    template <typename C> static two test(...);    

public:
    enum { value = sizeof(test<T>(0)) == sizeof(char) };
};


namespace rtt_ros_kdl_tools{
    
    template<typename T>
    class InOutPort
    {
    public:
        InOutPort(RTT::TaskContext * parent
            ,const std::string& port_name_input
            ,const std::string& port_name_output
            ,const std::string& doc_input=""
            ,const std::string& doc_output="")
        {
            parent->addPort(port_name_input,this->var_in).doc(doc_input);
            parent->addPort(port_name_output,this->var_out).doc(doc_output);
        }
        
        RTT::InputPort<T>& InOutPort::InputPort()
        {
            return this->port_in;
        }
        
        RTT::OutputPort<T>& InOutPort::OutputPort()
        {
            return this->port_out;
        }
        
        T& InOutPort::VarIn()
        {
            return this->var_in;
        }
        
        T& InOutPort::VarOut()
        {
            return this->var_out
        }
        
        void InOutPort::init(unsigned int vector_size)
        {
            this->var_in.setZero(vector_size);
            this->var_out.setZero(vector_size);
            this->port_in.setDataSample(this->var_in);
            this->port_out.setDataSample(this->var_out);
        }
        
    private:
        RTT::InputPort<T> port_in;
        
        T var_in,var_out;
                        
        RTT::OutputPort<T> port_out;
    };
}

#endif // __IN_OUT_PORT_HPP__