#include <libstkcomms/intelhex.hpp>

#include <chrono>
#include <fstream>
#include <iostream>

int main (int argc, char**argv) try {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <hexfile>\n";
        return 1;
    }

    auto input = std::ifstream(argv[1]);
    input.unsetf(std::ios::skipws);
    auto b = boost::spirit::istream_iterator{input};
    auto e = boost::spirit::istream_iterator{};

    auto intelHex = stk::intelhex::Grammar<decltype(b)>{};
    auto data = std::list<stk::intelhex::DataRecord>{};
    auto startTime = std::chrono::steady_clock::now();
    auto result = stk::qi::parse(b, e, intelHex, data);
    using FpSeconds = std::chrono::duration<double, std::ratio<1>>;
    auto elapsed = FpSeconds{std::chrono::steady_clock::now() - startTime};
    std::cout << "Parse complete after " << elapsed.count() << " seconds\n";
    if (result) {
        if (b == e) {
            std::cout << "Parsed " << data.size() << " data records\n";

        }
        else {
            std::cout << "It sorta worked, remaining: " << std::string(b, e) << "\n";
        }
    }
    else {
        std::cout << "Parse failed, yo\n";
    }
    return 0;
}
catch (std::exception& e) {
    std::cout << "Exception in main(): " << e.what() << "\n";
}