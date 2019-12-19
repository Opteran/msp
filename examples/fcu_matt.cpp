#include <FlightController.hpp>
#include <iostream>

const msp::FirmwareVariant fw_variant = msp::FirmwareVariant::BAFL; /* Betaflight */

/* Buffer into which asynchronous data will be written */
struct FcAsyncData{
	msp::msg::RawImu rawImu;
	msp::msg::Attitude attitude;
	msp::msg::Rc rc;
	
	FcAsyncData(msp::FirmwareVariant fw_v)
	: rawImu(fw_v)
	, attitude(fw_v)
	, rc(fw_v)
	{}

} fcAsyncData(msp::FirmwareVariant::BAFL);


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

    void onRawImu (const msp::msg::RawImu& rawImu) {
    	//msp::msg::ImuSI convimu(rawImu, 512.0, 1.0/4.096, 0.92f/10.0f, 9.80665f);

    	/* Convert raw IMU data to SI units, copy into global variable */
    	//fcAsyncData.imuSI = convimu;
	fcAsyncData.rawImu = rawImu;
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
        (argc > 1) ? std::string(argv[1]) : "/dev/ttyAMA0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;

    Callbacks cbs;
    fcu::FlightController fcu;
    fcu.connect(device, baudrate);

    // subscribe with custom type
    fcu.subscribe(&Callbacks::onIdent, &cbs, 1);
    fcu.subscribe(&Callbacks::onAttitude, &cbs, 1);
    fcu.subscribe(&Callbacks::onRawImu, &cbs, 1);
    fcu.subscribe(&Callbacks::onRC, &cbs, 1);

    while(1){
	std::cout << fcAsyncData.rawImu << std::endl;
    }

    // Ctrl+C to quit
    std::cin.get();
}
