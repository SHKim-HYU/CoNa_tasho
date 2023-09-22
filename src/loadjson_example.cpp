/*! 
 *  @file loadjson_example.cpp
 *  @brief Example code for loading json file
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Sep. 21. 2023
 *  @Comm
 */

// g++ loadjson_example.cpp -I/usr/include/jsoncpp -ljsoncpp -o ../bin/loadjson_example

#include <iostream>
#include <fstream>
#include <json/json.h>
#include "json_loader.h"

#ifndef CASADI_RESOURCE_PATH
#define CASADI_RESOURCE_PATH ""
#endif


int main() {
    std::string json_file_path = CASADI_RESOURCE_PATH;
    json_file_path += "tc.json";
    JsonLoader loader(json_file_path);

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
