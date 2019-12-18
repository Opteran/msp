#include <FlightController.hpp>
#include <iostream>

typedef struct fcAsyncData_t{
	msp::msg::ImuSI imuSI;
	msp::msg::Attitude attitude;
	msp::msg::Rc rc;
} fcAsyncData;

struct MyIdent : public msp::Message {
    MyIdent(msp::FirmwareVariant v) : Message(v) {}

    virtual msp::ID id() const override { return msp::ID::MSP_IDENT; }

    msp::ByteVector raw_data;

    virtual bool decode(const msp::ByteVector &data) override {
        raw_data = data;
        return true;
    }
};


struct Callbacks {
    void onIdent(const MyIdent &ident) {
        std::cout << "Raw Ident data: ";
        for(auto d : ident.raw_data) {
            std::cout << int(d) << " " << std::endl;
        }
    }

    void onImuRaw (const msp::msg::RawImu imuRaw) {
    	msp::msg::ImuSI convimu(imuRaw, 512.0, 1.0/4.096, 0.92f/10.0f, 9.80665f);

    	/* Convert raw IMU data to SI units, copy into global variable */
    	fcAsyncData.imuSI = convimu;
    }

    void onAttitude(const msp::msg::Attitude& attitude) {
    	fcAsyncData.attitude = attitude;
    }

    void onRC(const msp::msg::Rc& rc) {
        fcAsyncData.rc = rc;
    }

};

int main(int argc, char *argv[]) {
    const std::string device =
        (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;

    Callbacks cbs;
    fcu::FlightController fcu;
    fcu.connect(device, baudrate);

    // subscribe with custom type
    fcu.subscribe(&Callbacks::onIdent, &cbs, 1);
    fcu.subscribe(&Callbacks::onAttitude, &cbs, 1);
    fcu.subscribe(&Callbacks::onImuRaw, &cbs, 1);
    fcu.subscribe(&Callbacks::onRC, &cbs, 1);

    // Ctrl+C to quit
    std::cin.get();
}
