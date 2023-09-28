#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <string>
#include <iostream>


struct serial_message {
    int id;
    std::string data;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar & id;
        ar & data;
    }
};


class serial_port{
    public:
    serial_port(const std::string& port_location);
    ~serial_port();
    void easy_write_to_port(const std::string& servo_message);
    private:
    const std::string port_location;
    boost::asio::io_service io;
    boost::asio::serial_port port;
    const long BAUD_RATE = 9600;
    const long CHARACTER_SIZE = 8;
};