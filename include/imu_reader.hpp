#pragma once
#include "types.hpp"

class ImuReader {
public:
    explicit ImuReader(const std::string& drive_path);

    std::vector<ImuMeasurement> measurements() const;
    std::vector<double> loadTimestamps() const;
    
private:
    std::string drive_path_;
    std::vector<ImuMeasurement> parseOxts() const;
};