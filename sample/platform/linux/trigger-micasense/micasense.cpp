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
#include "micasense.hpp"

void ImageCapture::captureAndSyncImages() {
    CURL *curl;
    CURLcode res;
    std::string response;
    std::string capture_request = "/capture?store_capture=true&cache_jpeg=" + band_binary + "&cache_raw=" + band_binary + "&block=true";

    // URL to capture and cache all 5 bands in JPEG and TIFF formats
    std::string capture_url = "http://" + micasense_ip + capture_request;

    // Initialize cURL
    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();

    if (curl) {
        curl_easy_setopt(curl, CURLOPT_URL, capture_url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, StringCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

        // Perform the capture request
        res = curl_easy_perform(curl);

        // Check for errors
        if (res != CURLE_OK) {
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
        }
        else {
            // Parse the string into a JSON object
            nlohmann::json json_obj = nlohmann::json::parse(response);

            // Get the current timestamp
            std::string timestamp = getCurrentTimestamp();

            // Access JPEG storage path
            for (auto& jpeg_path : json_obj["jpeg_storage_path"]) {
                std::cout << jpeg_path << "\n";

                // Make a request to the JPEG URL 
                std::vector<unsigned char> jpeg_data;
                std::string jpeg_url = "http://" + micasense_ip + jpeg_path.get<std::string>();
                curl_easy_setopt(curl, CURLOPT_URL, jpeg_url.c_str());
                curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, VectorCallback);
                curl_easy_setopt(curl, CURLOPT_WRITEDATA, &jpeg_data);
                res = curl_easy_perform(curl);

                if (res == CURLE_OK) {
                    // Extract the filename from the path
                    std::string jpeg_path_str = jpeg_path.get<std::string>();
                    size_t pos = jpeg_path_str.find_last_of('/');
                    std::string filename;

                    if (pos != std::string::npos) {
                        filename = jpeg_path_str.substr(pos + 1);
                    }

                    // Append the timestamp to the filename
                    filename = timestamp + "_" + filename;

                    // Save the data
                    std::ofstream jpeg_file("/home/robotics/dropbox/micasense_images/jpeg/" + filename, std::ios::binary);
                    jpeg_file.write(reinterpret_cast<const char*>(jpeg_data.data()), jpeg_data.size());
                    jpeg_file.close();
                }
                jpeg_data.clear();
            }

            // Access TIFF storage path
            for (auto& tiff_path : json_obj["raw_storage_path"]) {
                std::cout << tiff_path << "\n";

                // Make a request to the JPEG URL 
                std::vector<unsigned char> tiff_data;
                std::string tiff_url = "http://" + micasense_ip + tiff_path.get<std::string>();
                curl_easy_setopt(curl, CURLOPT_URL, tiff_url.c_str());
                curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, VectorCallback);
                curl_easy_setopt(curl, CURLOPT_WRITEDATA, &tiff_data);
                res = curl_easy_perform(curl);

                if (res == CURLE_OK) {
                    // Extract the filename from the path
                    std::string tiff_path_str = tiff_path.get<std::string>();
                    size_t pos = tiff_path_str.find_last_of('/');
                    std::string filename;

                    if (pos != std::string::npos) {
                        filename = tiff_path_str.substr(pos + 1);
                    }

                    // Append the timestamp to the filename
                    filename = timestamp + "_" + filename;

                    // Save the data
                    std::ofstream tiff_file("/home/robotics/dropbox/micasense_images/tiff/" + filename, std::ios::binary);
                    tiff_file.write(reinterpret_cast<const char*>(tiff_data.data()), tiff_data.size());
                    tiff_file.close();
                }
                tiff_data.clear();
            }
        }
        // Cleanup
        curl_easy_cleanup(curl);
    }

    curl_global_cleanup();

    // Sync images to dropbox
    int result = system("rclonesync --first-sync airas:/micasense_images ~/dropbox/micasense_images");
    if (result != 0) {
        // Handle error
        std::cerr << "Error running rclonesync command. Return code: " << result << std::endl;
    }
    else {
        std::cout << "Images synced to dropbox" << std::endl;
    }
}

size_t ImageCapture::VectorCallback(void* contents, size_t size, size_t nmemb, std::vector<unsigned char>* s) {
    size_t newLength = size * nmemb;
    try {
        s->insert(s->end(), (unsigned char*)contents, (unsigned char*)contents + newLength);
    }
    catch (std::bad_alloc& e) {
        // handle memory problem
        return 0;
    }
    return newLength;
}

size_t ImageCapture::StringCallback(void* contents, size_t size, size_t nmemb, std::string* s) {
    size_t newLength = size * nmemb;
    try {
        s->append((char*)contents, newLength);
    }
    catch (std::bad_alloc& e) {
        // handle memory problem
        return 0;
    }
    return newLength;
}

std::string ImageCapture::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm* ltm = std::localtime(&now_time);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d%H%M", ltm);
    return std::string(buffer);
}

