#include <libstkcomms/intelhex.hpp>

#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <boost/iostreams/device/mapped_file.hpp>

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#define SPIRIT 0
#define INTERPROCESS 0
#define MAPPED_FILE 0
#define VECTOR 0
#define RDBUF 1

int main (int argc, char**argv) try {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <hexfile>\n";
        return 1;
    }

#if SPIRIT
    auto input = std::ifstream(argv[1]);
    input.unsetf(std::ios::skipws);
    auto b = boost::spirit::istream_iterator{input};
    auto e = boost::spirit::istream_iterator{};
#elif INTERPROCESS
    namespace ip = boost::interprocess;
    auto file = ip::file_mapping{argv[1], ip::read_only};
    auto region = ip::mapped_region{file, ip::read_only};
    auto b = static_cast<uint8_t*>(region.get_address());
    auto e = b + region.get_size();
#elif MAPPED_FILE
    using boost::iostreams::mapped_file_source;
    auto file = mapped_file_source{argv[1]};
    auto b = file.data();
    auto e = b + file.size();
#elif VECTOR
    std::ifstream t(argv[1]);
    t.seekg(0, std::ios::end);
    auto size = t.tellg();
    std::string buffer(size, ' ');
    t.seekg(0);
    t.read(&buffer[0], size);
    auto b = buffer.cbegin();
    auto e = buffer.cend();
#elif RDBUF
    std::ifstream t(argv[1]);
    std::stringstream buffer;
    buffer << t.rdbuf();
    auto s = buffer.str();
    auto b = s.cbegin();
    auto e = s.cend();
#endif

    auto intelHex = stk::intelhex::Grammar<decltype(b)>{};
    auto data = std::list<stk::intelhex::DataRecord>{};
    auto startTime = std::chrono::steady_clock::now();
    auto result = stk::qi::parse(b, e, intelHex >> stk::qi::eoi, data);
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