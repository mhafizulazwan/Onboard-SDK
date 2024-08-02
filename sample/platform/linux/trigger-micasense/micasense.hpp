#ifndef MICASENSE_HPP
#define MICASENSE_HPP

#include <iostream>
#include <curl/curl.h>
#include <vector>
#include <iomanip>
#include <fstream>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <cstdlib>
#include <chrono>
#include <ctime>

class ImageCapture {
public:
    void captureImages();
    void syncImagesToDropbox();

private:
    std::string micasense_ip = "192.168.0.85";
    std::string band_binary = "7"; // 7 is the binary representation of 3 bands (00111)
    static size_t VectorCallback(void* contents, size_t size, size_t nmemb, std::vector<unsigned char>* s);
    static size_t StringCallback(void* contents, size_t size, size_t nmemb, std::string* s);
    static std::string getCurrentTimestamp();
};

#endif // MICASENSE_HPP