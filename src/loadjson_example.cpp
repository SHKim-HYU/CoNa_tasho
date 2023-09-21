/*! 
 *  @file loadjson_example.cpp
 *  @brief Example code for loading json file
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Sep. 21. 2023
 *  @Comm
 */

// g++ loadjson_example.cpp -I/usr/include/jsoncpp -ljsoncpp -o output_executable

#include <iostream>
#include <fstream>
#include <json/json.h>

class JsonLoader {
private:
    Json::Value data;

public:
    // Constructor: Takes the path of the JSON file as an argument.
    JsonLoader(const std::string &filePath) {
        std::ifstream input_file(filePath);
        if (!input_file.is_open()) {
            std::cerr << "Failed to open JSON file." << std::endl;
            return;
        }

        Json::CharReaderBuilder rbuilder;
        std::string errs;
        bool parsingSuccessful = Json::parseFromStream(rbuilder, input_file, &data, &errs);
        if (!parsingSuccessful) {
            std::cerr << "Failed to parse JSON: " << errs << std::endl;
            return;
        }

        input_file.close();
    }

    // Function to fetch the value corresponding to a specific key in the JSON object.
    Json::Value getValue(const std::string &key) const {
        if (data.isMember(key)) {
            return data[key];
        }
        return Json::Value(); 
    }
};

int main() {
    JsonLoader loader("../lib/casadi/tc.json");

    // Fetch and print the value for the key "x0".
    Json::Value x0 = loader.getValue("x0");

    if (!x0.isNull()) {
        std::cout << "x0: " << x0.toStyledString() << std::endl;
        std::cout << "end: " << x0["end"].asInt() << std::endl;
        std::cout << "start: " << x0["start"].asInt() << std::endl;
    }

    // Fetch and print the value for the key "y0".
    Json::Value y0 = loader.getValue("y0");
    if (!y0.isNull()) {
        std::cout << "y0: " << y0.toStyledString() << std::endl;
    }

    // Fetch and print the value for the key "th0".
    Json::Value th0 = loader.getValue("th0");
    if (!th0.isNull()) {
        std::cout << "th0: " << th0.toStyledString() << std::endl;
    }

    // Fetch and print the value for the key "mpc_fun_name" and "mpc_file".
    Json::Value mpc_fun_name = loader.getValue("mpc_fun_name");
    Json::Value mpc_file = loader.getValue("mpc_file");
    if (!mpc_fun_name.isNull()) {
        std::cout << "mpc_fun_name: " << mpc_fun_name.asString() << std::endl;
    }
    if (!mpc_file.isNull()) {
        std::cout << "mpc_file: " << mpc_file.asString() << std::endl;
    }    
    return 0;
}
