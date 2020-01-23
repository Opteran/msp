#include <FlightController.hpp>
#include <iostream>
#include <chrono>

const msp::FirmwareVariant fw_variant = msp::FirmwareVariant::BAFL; /* Betaflight */

#define ASYNCHRONOUS_UPDATE_RATE (0.001)

/* Message counters */
struct Counters{
    uint32_t rawImu, attitude, rc;
};

static Counters counters;

struct Callbacks {

    void onRawImu (const msp::msg::RawImu& rawImu __attribute__((unused))) {
	counters.rawImu++;
    }

    void onAttitude(const msp::msg::Attitude& attitude __attribute__((unused))) {
    	counters.attitude++;
    }

    void onRC(const msp::msg::Rc& rc __attribute__((unused))) {
        counters.rc++;
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
    fcu.subscribe(&Callbacks::onAttitude, &cbs, ASYNCHRONOUS_UPDATE_RATE);
    fcu.subscribe(&Callbacks::onRawImu, &cbs, ASYNCHRONOUS_UPDATE_RATE);
    fcu.subscribe(&Callbacks::onRC, &cbs, ASYNCHRONOUS_UPDATE_RATE);

    std::cout << "Subscription complete" << std::endl;

    Counters countersPrev = {.rawImu = 0, .attitude = 0, .rc = 0};
    std::chrono::steady_clock::time_point timePrev = std::chrono::steady_clock::now();

    while(1){
        std::chrono::steady_clock::time_point timeNow = std::chrono::steady_clock::now();
        float dt = 1.0e-6*std::chrono::duration_cast<std::chrono::microseconds>(timeNow - timePrev).count();
        Counters countRate = {
            .rawImu = uint32_t((counters.rawImu - countersPrev.rawImu) / dt),
            .attitude = uint32_t((counters.attitude - countersPrev.attitude) / dt),
            .rc = uint32_t((counters.rc - countersPrev.rc) / dt)
        };

        countersPrev = counters;
        timePrev = timeNow;

	std::cout << "Message rates " << countRate.rawImu << "\t" << countRate.attitude << "\t" << countRate.rc << std::endl;
        sleep(1);
    }

    // Ctrl+C to quit
    std::cin.get();
}
