/*
 * opendlv-device-lidar-vlp32c decodes VLP32c data realized for OpenDLV.
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "vlp32c-decoder.hpp"

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("vlp32c_port")) || (0 == commandlineArguments.count("cid")) ) {
        std::cerr << argv[0] << " decodes pointcloud data from a VelodyneLidar VLP32c unit and publishes it to a running OpenDaVINCI session using the OpenDLV Standard Message Set." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " [--vlp32c_ip=<IPv4-address>] --vlp32c_port=<port> --cid=<OpenDaVINCI session> [--id=<Identifier in case of multiple lidars>] [--verbose] [--intensity=<number of higher bits used for intensity>]" << std::endl;
        std::cerr << "         --intensity: VLP32c is using 16 bits to encode distances by default; when specifying this parameter with" << std::endl;
        std::cerr << "                      a value n > 0, the higher n bits will be used to encode intensity values for a given" << std::endl;
        std::cerr << "                      distance and thus, not using these n bits for distances. Thus, specifying this" << std::endl;
        std::cerr << "                      parameter might impose a *SAFETY RISK* as this software would not report objects" << std::endl;
        std::cerr << "                      that are within that particular range to the sensor. USE THIS PARAMETER AT YOUR OWN RISK!!" << std::endl;
        std::cerr << "                      Possible value range is [0..6]." << std::endl;
        std::cerr << "Example: " << argv[0] << " --vlp32c_ip=0.0.0.0 --vlp32c_port=2368 --cid=111" << std::endl;
        retCode = 1;
    } else {
        const uint32_t ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
        int32_t INTENSITY{(commandlineArguments["intensity"].size() != 0) ? static_cast<int32_t>(std::stoi(commandlineArguments["intensity"])) : 0};
        if ( (INTENSITY < 0) || (INTENSITY > 6) ) {
            std::cerr << "[lidar-vlp32c] Specified value for intensity is not within the range [0..6]; using 0." << std::endl;
            INTENSITY = 0;
        }
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Interface to a running OpenDaVINCI session (ignoring any incoming Envelopes).
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"])),
            [](auto){}
        };

        // Interface to VelodyneLidar VLP32c unit.
        const std::string VLP32E_ADDRESS((commandlineArguments.count("vlp32c_ip") == 0) ? "0.0.0.0" : commandlineArguments["vlp32c_ip"]);
        const uint32_t VLP32E_PORT(std::stoi(commandlineArguments["vlp32c_port"]));
        VLP32cDecoder vlp32cDecoder(INTENSITY);
        cluon::UDPReceiver fromDevice(VLP32E_ADDRESS, VLP32E_PORT,
            [&od4Session = od4, &decoder = vlp32cDecoder, senderStamp = ID, VERBOSE](std::string &&d, std::string &&/*from*/, std::chrono::system_clock::time_point &&tp) noexcept {
            auto retVal = decoder.decode(d);
            if (!retVal.empty()) {
                cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);

                for(auto e : retVal) {
                    od4Session.send(e, sampleTime, senderStamp);
                }

                // Print values on console.
                if (VERBOSE) {
                    std::cout << "[lidar-vlp32c] Decoded data into PointCloudReading." << std::endl;
                }
            }
        });

        // Just sleep as this microservice is data driven.
        using namespace std::literals::chrono_literals;
        while (od4.isRunning()) {
            std::this_thread::sleep_for(1s);
        }
    }
    return retCode;
}
