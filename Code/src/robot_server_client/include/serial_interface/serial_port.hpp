#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <string>
#include <iostream>

/**
 * @brief struct of a serial message. 
 * 
 */
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
/**
 * @brief Construct a new serial port object
 * 
 * @param port_location 
 */
serial_port(const std::string& port_location);
/**
 * @brief Destroy the serial port object
 * 
 */
~serial_port();
/**
 * @brief writes a message to the serial port
 * 
 * @param servo_message 
 */
void easy_write_to_port(const std::string& servo_message);
private:
/**
 * @brief contains the directory of the port
 * 
 */
const std::string port_location;
/**
 * @brief current io_service
 * 
 */
boost::asio::io_service io;
/**
 * @brief Contains the instance of the port
 * 
 */
boost::asio::serial_port port;
/**
 * @brief contains the set baud rate
 * 
 */
const long BAUD_RATE = 115200;
/**
 * @brief contains set character size
 * 
 */
const long CHARACTER_SIZE = 8;
};