#ifndef MSP_HPP
#define MSP_HPP

#include "SerialPort.hpp"
#include "types.hpp"

#include <stdexcept>
#include <chrono>

namespace msp {

// exception to throw when header contains wrong data
class MalformedHeader : public std::runtime_error {
public:
    MalformedHeader(const uint8_t exp, const uint8_t rcv)
        : std::runtime_error(
              "Malformed header: "
              "expected "+std::to_string(exp)+" ("+std::string(1,char(exp))+"), "
              "received "+std::to_string(rcv)+" ("+std::string(1,char(rcv))+")") {}
};

class UnknownMsgId : public std::runtime_error {
public:
    UnknownMsgId(uint8_t id) : runtime_error(
        "Unknown MSP id! FC refused to process message with id: "+std::to_string(id))
    { }
};

// exception to throw if reported CRC does not match with computed
class WrongCRC : public std::runtime_error {
public:
    WrongCRC(const msp::ID msg_id, const uint8_t exp, const uint8_t rcv)
        : std::runtime_error(
              "CRC not matching: "
              "Message "+std::to_string(uint(msg_id))+", "
              "expected CRC "+std::to_string(exp)+", "
              "received CRC "+std::to_string(rcv))
    { }
};

class NoData : public std::runtime_error {
public:
    NoData() : std::runtime_error("No data available!") { }
};

/**
 * @brief The DataID struct
 */
struct DataID {
    ByteVector data;    //!< rawdata vector
    ID id;              //!< message ID

    /**
     * @brief DataID
     * @param data vector of raw data bytes
     * @param id message ID
     */
    DataID(ByteVector data, ID id) : data(data), id(id) {}
};

/**
 * @brief The MSP class
 */
class MSP {
public:

    /**
     * @brief MSP construct MSP communication without establishing a connection
     */
    MSP();

    /**
     * @brief MSP constructor for MSP communication
     * @param device device path
     * @param baudrate serial baudrate
     */
    MSP(const std::string &device, const uint baudrate=115200);

    /**
     * @brief connect establish connection to serial device
     * @param device path or name of serial device
     * @param baudrate serial baudrate (default: 115200)
     * @return true on success
     */
    bool connect(const std::string &device, const uint baudrate=115200);

    /**
     * @brief request send command and request data from FC once
     * @param request request message
     * @return true on success
     * @return false on failure
     */
    bool request(msp::Request &request);

    /**
     * @brief request_block continuously send command and request data until data has been received
     * @param request request message
     * @return true when data has been received
     */
    bool request_block(msp::Request &request);

    /**
     * @brief request_wait wait for data while continuously sending command
     * @param request request message
     * @param wait_ms waiting time in between sending request and receiving response
     * @return true when data has been received
     */
    bool request_wait(msp::Request &request, uint wait_ms);

    /**
     * @brief respond send data to FC and read acknowledge
     * @param response response message
     * @return true on success
     * @return false on failure
     */
    bool respond(const msp::Response &response);

    /**
     * @brief respond_block send data to FC until acknowledge has been received
     * @param response response message with data
     * @return true when acknowledge has been received
     */
    bool respond_block(const msp::Response &response);

    /**
     * @brief sendData send raw data and ID to flight controller
     * @param id message ID
     * @param data raw data
     * @return true on success
     * @return false on failure
     */
    bool sendData(const ID id, const ByteVector &data = ByteVector(0));

    /**
     * @brief send encode message and send payload
     * @param response message sent to FC
     * @return true on success
     * @return false on failure
     */
    bool send(const msp::Response &response) {
        return sendData(response.id(), response.encode());
    }

    /**
     * @brief receiveData receive raw data from flight controller
     * @return pair of data and message ID
     */
    DataID receiveData();

    /**
     * @brief setWait set time (microseconds) between sending and receiving
     * After sending a request to the FC, we need to wait a small amount of time
     * for the FC to process our request and to respond.
     *
     * @param wait_us waiting time in microseconds
     */
    void setWait(unsigned int wait_us) {
        wait = wait_us;
    }

private:
    /**
     * @brief crc compute checksum of data package
     * @param id message ID
     * @param data raw data vector
     * @return checksum
     */
    uint8_t crc(const ID id, const ByteVector &data);

    SerialPort sp;      //!< serial port
    unsigned int wait;  //!< time (micro seconds) to wait before waiting for response
};

} // namespace msp

#endif // MSP_HPP
