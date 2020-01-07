#include <FlightController.hpp>
#include <iostream>

const msp::FirmwareVariant fw_variant = msp::FirmwareVariant::BAFL; /* Betaflight */

/* Buffer into which asynchronous data will be written */
struct FcAsyncData{
    msp::msg::RawImu rawImu;
    msp::msg::Attitude attitude;
    msp::msg::Rc rc;
    msp::msg::RcBeforeOverride rcBeforeOverride;
    
    FcAsyncData(msp::FirmwareVariant fw_v)
        : rawImu(fw_v)
        , attitude(fw_v)
        , rc(fw_v)
        , rcBeforeOverride(fw_v)
        {}

    msp::msg::ImuSI imuSI(const float acc_1g,     // sensor value at 1g
        const float gyro_unit,  // resolution in 1/(deg/s)
        const float magn_gain,  // scale magnetic value to uT (micro Tesla)
        const float si_unit_1g  // acceleration at 1g (in m/s^2))
    ) {
        return msp::msg::ImuSI(rawImu, acc_1g, gyro_unit, magn_gain, si_unit_1g);
    }

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
	fcAsyncData.rawImu = rawImu;
    }

    void onAttitude(const msp::msg::Attitude& attitude) {
    	fcAsyncData.attitude = attitude;
    }

    void onRC(const msp::msg::Rc& rc) {
        fcAsyncData.rc = rc;
    }
    void onRCBeforeOverride(const msp::msg::RcBeforeOverride& rcBeforeOverride) {
        fcAsyncData.rcBeforeOverride = rcBeforeOverride;
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
    fcu.subscribe(&Callbacks::onAttitude, &cbs, 0.01);
    fcu.subscribe(&Callbacks::onRawImu, &cbs, 0.01);
    fcu.subscribe(&Callbacks::onRC, &cbs, 0.01);
    fcu.subscribe(&Callbacks::onRCBeforeOverride, &cbs, 0.01);

    std::cout << "Subscription complete" << std::endl;

    while(1){
	for(int i=0; i<20; i++)
	{
		std::cout << fcAsyncData.rc << fcAsyncData.rcBeforeOverride << std::endl;

		std::vector<unsigned short> rcToSet(16,1234);
		fcu.setRc(rcToSet);
		usleep(100000);
	}
	for(int i=0; i<20; i++)
	{
		std::cout << fcAsyncData.rc << fcAsyncData.rcBeforeOverride << std::endl;

		usleep(100000);
	}
    }

    // Ctrl+C to quit
    std::cin.get();
}
