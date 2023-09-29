 #include "serial_port.hpp"
 
serial_port::serial_port(const std::string& port_location) : port_location(port_location), port(io, port_location)
{
    port.set_option(boost::asio::serial_port_base::baud_rate(BAUD_RATE));
    port.set_option(boost::asio::serial_port_base::character_size(CHARACTER_SIZE));
    port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
}


serial_port::~serial_port()
{
    port.close();
}


void serial_port::easy_write_to_port(const std::string& servo_message)
{
    boost::asio::async_write(port, boost::asio::buffer(servo_message),
         [&](const boost::system::error_code& error, std::size_t bytes_transferred) {
             if (error) {
                 std::cerr << "Error sending message: " << error.message() << "Total bytes transferred" << bytes_transferred << std::endl;
             }
         });
}
